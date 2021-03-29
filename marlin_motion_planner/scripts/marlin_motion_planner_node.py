#! /usr/bin/env python

import actionlib
import marlin_motion_planner.msg
import marlin_serial.msg
import rospy
import numpy as np
from  dijkstra_dev_to import Graph, ChessPos, make_edge

class GCODEComposer:
    def __init__(self, dwell = 200):
        self._cmd_buffer = []
        self._dwell = dwell
        
    def ApplyMarlinOffsetScaled(self, target):
        # TODO: load offsets from config
        offsets = [0.0, 25.0]
        retVal = []
        retVal.append(target[0] * 30.0 + offsets[0])
        retVal.append(target[1] * 30.0 + offsets[1])
        return retVal

    def ActivateMagnet(self, state = False):
        if state == True:
            self._cmd_buffer.append('M106')
            if self._dwell > 0:
                self._cmd_buffer.append('G4 P{0}'.format(self._dwell))
        else:
            self._cmd_buffer.append('M107')
            

    def MoveEffectorTo(self, target):
        marlin_target = self.ApplyMarlinOffsetScaled(target)
        #cmd = 'G1 X{0} Y{1} F{2} ; ({3})'.format(marlin_target[0], marlin_target[1], 3500, target) # feedrate?
        cmd = 'G1 X{0} Y{1} F{2}'.format(marlin_target[0], marlin_target[1], 3500) # feedrate?
        self._cmd_buffer.append(cmd)

    def MoveEffectorAlongPath(self, path):
        self.MoveEffectorTo([path[0].x, path[0].y])
        self.ActivateMagnet(True)
        for segment in path[1:]:
            self.MoveEffectorTo([segment.x, segment.y])
        self.ActivateMagnet(False)

    def LogCommands(self):
        rospy.loginfo('Marlin commands: {0}'.format(len(self._cmd_buffer)))
        for (idx, cmd) in enumerate(self._cmd_buffer):
            rospy.loginfo('{0} : {1}'.format(idx, cmd))

    def MoveCommandQueue(self, source, target):
        self.MoveEffectorTo([source.x, source.y])
        self.ActivateMagnet(True)
        self.MoveEffectorTo([target.x, target.y])
        self.ActivateMagnet(False)
    
    def GetBuffer(self):
        return self._cmd_buffer


def is_boxed_in(point, state, with_defeated):
    free_slots = [n for n in point.get_neighbors() if state[n.to_idx(with_defeated)] is False]
    return len(free_slots) == 0

def calc_neighbor_cost(pos, state, with_defeated ):
    boxed_in = True
    neighbor_cost = []
    for n in pos.get_neighbors():
        cost = 1000
        if state[n.to_idx(with_defeated)] is False:
            cost = pos.euclidian(n)
        neighbor_cost.append(make_edge(pos, n, cost))
    return neighbor_cost

#def check_intermediate_move(self, point):

def path_merge_delta(path):
    path_merged = []
    last_delta = (0,0)
    node_prev = path[0]
    for node in path:
        cur_delta = node.delta(node_prev)
        if cur_delta != last_delta:
            path_merged.append(node_prev)
            last_delta = cur_delta
        #else:
        #    rospy.loginfo('merging edge {0}: {1}'.format(node, cur_delta))
        node_prev = node

    path_merged.append(path[-1])

    return path_merged


def pathfinding(source, target, state, with_defeated = False):
    data = []
    positions = []

    board_size_x = 8
    board_size_y = 8
    pos_offset = 0

    # Move defeated pieces to the side of the board
    if with_defeated is True:
        board_size_x = 10
        source.x += 1
        target.x += 1
        #pos_offset = -1

    #check if start is boxed in
    if is_boxed_in(source, state, with_defeated):
        rospy.logerr('Start node is boxed in: {0}'.format(source))

    if is_boxed_in(target, state, with_defeated):
        rospy.logerr('End node is boxed in: {0}'.format(target))

    #if with_defeated == True:
    #    str = ''
    #    for y in range(board_size_y):
    #        for x in range(board_size_x):
    #            idx = x * 10 + y
    #            st = int(state[idx])
    #            str += '{0} '.format(st)
    #        str += '\n'
    #    print(str)

    neighbours_cost = [None] * 100
    for x in range(board_size_x):
        for y in range(board_size_y):
            pos = ChessPos(x + pos_offset, y, with_defeated)
            positions.append(pos)
            neighbours_cost[pos.to_idx(with_defeated)] = calc_neighbor_cost(pos, state, with_defeated)

    
    #if with_defeated is True:
    #    print(neighbours_cost[0])

    #graph_flat = [edge for node_list in neighbours_cost for edge in node_list if node_list is not None]
    graph_flat = []
    for node_list in neighbours_cost:
        if node_list is not None:
            for edge in node_list:
                graph_flat.append(edge)

    graph = Graph(graph_flat)
    
    res, path_cost = graph.dijkstra(source, target)
    if len(res) is 0:
        rospy.logerr("graph planning failed!")

    #if with_defeated is True:
    #    for c in path_cost:
    #        print(c) 

    conflicting_cnt = sum(1 for x in path_cost if x.cost == 1000)
    # TODO: perform dijkstra again if too many conflicting nodes
    if conflicting_cnt > 0:
        rospy.logwarn('Conflicting nodes: {0}'.format(conflicting_cnt))

    #intermediate moves if slot is blocked
    intermediate_moves = []
    for edge in path_cost:
        if edge.cost == 1000:
            rospy.loginfo('processing edge with high cost {0}'.format(edge.end))
            # get node neighbours excluding path points
            neighbours = [n for n in neighbours_cost[edge.end.to_idx(with_defeated)] if n.end not in list(res) and n.cost < 1000]

            neighbours_nontaken = [n for n in neighbours if n.end not in intermediate_moves]

            if len(neighbours_nontaken) == 0:
                rospy.logerr('neighbour nodes blocked by intermediate')

            if len(neighbours) == 0:
                rospy.logerr('No free neighbour slots')

            #prefer non-diagonal moves
            neighbours_rect = [n for n in neighbours if n.cost <= 1.5]
            if len(neighbours_rect) > 0:
                intermediate_moves.append(neighbours_rect[0])
            else:
                intermediate_moves.append(neighbours[0])

    if len(intermediate_moves) > 0:
        rospy.logwarn('path: {0}'.format(res))
        rospy.logwarn('Intermediate moves: {0}'.format(intermediate_moves))

    #if with_defeated is True:
    #    print(res)
    return res, path_cost, intermediate_moves

class PieceMoveAction:
    _feedback = marlin_motion_planner.msg.PieceMoveActionFeedback()
    _result = marlin_motion_planner.msg.PieceMoveActionResult()

    def __init__(self, name):
        self._action_name = name
        self._gcode_ac = actionlib.SimpleActionClient('gcode', marlin_serial.msg.GcodeAction)
        self._cmd = []
        self._as = actionlib.SimpleActionServer(name, marlin_motion_planner.msg.PieceMoveAction,
        execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        success = True

        gc = GCODEComposer(dwell = 200)

        state = goal.state.states
        sourcePos = ChessPos(goal.source[0], goal.source[1], False)
        targetPos = ChessPos(goal.target[0], goal.target[1], False)
        # clear source from state array
        state[sourcePos.to_idx()] = False

        result, path_cost, intermediate = pathfinding(sourcePos, targetPos, state, False)

        result_merged = path_merge_delta(result)

        for move in intermediate:
            gc.MoveCommandQueue(move.start, move.end)

        gc.MoveEffectorAlongPath(result_merged)

        for move in intermediate:
            gc.MoveCommandQueue(move.end, move.start)

        gc.LogCommands()

        goal = marlin_serial.msg.GcodeGoal(commands=gc.GetBuffer())
        self._gcode_ac.send_goal(goal)
        self._gcode_ac.wait_for_result()
        # TODO: Fixed neutral position?
        if success:
            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def execute_cb_new(self, goal):

        success = True

        gc = GCODEComposer(dwell = 200)

        state = goal.state.states
        sourcePos = ChessPos(goal.source[0], goal.source[1])
        targetPos = ChessPos(goal.target[0], goal.target[1])
        # clear source from state array
        state[sourcePos.to_idx()] = False

        data = []
        positions = []

        #check if start is boxed in
        if self.is_boxed_in(sourcePos, state):
            rospy.logerr('Start node is boxed in: {0}'.format(sourcePos))

        if self.is_boxed_in(targetPos, state):
            rospy.logerr('End node is boxed in: {0}'.format(sourcePos))

        # clear start and end points 
        # calculate dijkstra
        # check path for obstrucions
            # cost > 1000 -> boxed in
            # for each boxed in node:
                # blacklist valid path points
                # try move to neighboring field
                # store intermediate
        # restore moves

        neighbours_cost = []
        for x in range(8):
            for y in range(8):
                pos = ChessPos(x, y)
                positions.append(pos)

                #neighbour_edges = [self.calc_neighbor_cost(pos, state) for n in pos.get_neighbors()]
                #print ('Position: {0} : {1}'.format(pos, neighbour_edges))
                neighbours_cost.append(self.calc_neighbor_cost(pos, state))
        #print (data)
        #print(neighbours_cost)
        #data = sum(neighbours_cost)
        graph_flat = [edge for node_list in neighbours_cost for edge in node_list]
        #print(graph_flat)
        graph = Graph(graph_flat)
        
        res, path_cost = graph.dijkstra(sourcePos, targetPos)
        if len(res) is 0:
            rospy.logerr("graph planning failed!")

        #print(res)
        #print(path_cost)

        #intermediate moves if slot is blocked
        intermediate_moves = []
        for edge in path_cost:
            if edge.cost == 1000:
                rospy.loginfo('processing edge with high cost {0}'.format(edge.end))
                # get node neighbours excluding path points
                neighbours = [n for n in neighbours_cost[edge.end.to_idx()] if n.end not in list(res) and n.cost < 1000]

                if len(neighbours) == 0:
                    rospy.logerr('Unable to move!')

                #prefer non-diagonal moves
                neighbours_rect = [n for n in neighbours if n.cost <= 1.5]
                if len(neighbours_rect) > 0:
                    intermediate_moves.append(neighbours_rect[0])
                else:
                    intermediate_moves.append(neighbours[0])

                #print(neighbours)
                #print(neighbours_rect)
                #print(res)
                #print(len(neighbours) - len(res))

        if len(intermediate_moves) > 0:
            rospy.logwarn('path: {0}'.format(res))
            rospy.logwarn('Intermediate moves: {0}'.format(intermediate_moves))

            


        for move in intermediate_moves:
            gc.MoveCommandQueue(move.start, move.end)

        gc.MoveCommandQueue(sourcePos, targetPos)

        for move in intermediate_moves:
            gc.MoveCommandQueue(move.end, move.start)

        gc.LogCommands()

        goal = marlin_serial.msg.GcodeGoal(commands=self._cmd)
        # TODO: Fixed neutral position?
        if success:
            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


    def execute_cb_old(self, goal):
        r = rospy.Rate(10)
        success = True

        gc = GCODEComposer(dwell = 200)

        #motion planning

        state = goal.state.states
        sourcePos = ChessPos(goal.source[0], goal.source[1])
        targetPos = ChessPos(goal.target[0], goal.target[1])
        # clear source from state array
        state[sourcePos.to_idx()] = False



        #print(goal.state)
        data = []
        positions = []


        #check if start is boxed in
        if self.is_boxed_in(sourcePos, state):
            rospy.logerr('Start node is boxed in: {0}'.format(sourcePos))

        if self.is_boxed_in(targetPos, state):
            rospy.logerr('End node is boxed in: {0}'.format(sourcePos))

        for x in range(8):
            for y in range(8):
                pos = ChessPos(x, y)
                positions.append(pos)






                #print('neighbours for: {0}'.format(pos))
                #if self.is_boxed_in(pos, state):
                    #rospy.logerr('Node boxed in: {0}'.format(pos))
                for n in pos.get_neighbors():
                    cost = pos.euclidian(n)
                    #print(cost)
                    #if goal.state.states[n.x*]
                    if goal.state.states[n.to_idx()] is True and n != ChessPos(goal.source[0], goal.source[1]):
                        cost = 1000
                    #print('neighbour: {0} cost: {1}'.format(n, cost))
                    data.append(make_edge(pos, n, cost))
        #print (data)

        graph = Graph(data)
        #graph = Graph([
        #    ("a", "b", 7),  ("a", "c", 9),  ("a", "f", 14), ("b", "c", 10),
        #    ("b", "d", 15), ("c", "d", 11), ("c", "f", 2),  ("d", "e", 6),
        #    ("e", "f", 9)])
        res = graph.dijkstra(ChessPos(goal.source[0], goal.source[1]), ChessPos(goal.target[0], goal.target[1]))
        if len(res) is 0:
            rospy.loger("graph planning failed!")

        gc.MoveEffectorTo(goal.source)
        gc.ActivateMagnet(True)
        gc.MoveEffectorTo(goal.target)
        gc.ActivateMagnet(False)
        gc.LogCommands()

        # move to source
        #self._cmd.append(MoveEffectorTo(goal.source))
        # activate magnet
        #self._cmd.append(ActivateMagnet(True))
        # move to target
        #self._cmd.append(MoveEffectorTo(goal.target))
        # deactivate magnet
        #self._cmd.append(ActivateMagnet(False))

        goal = marlin_serial.msg.GcodeGoal(commands=self._cmd)
        #self._gcode_ac.send_goal(goal)
        # TODO: Fixed neutral position?
        if success:
            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class PieceDefeatedAction:
    _feedback = marlin_motion_planner.msg.PieceDefeatedActionFeedback()
    _result = marlin_motion_planner.msg.PieceDefeatedActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(name, marlin_motion_planner.msg.PieceDefeatedAction,
        execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):

        success = True

        gc = GCODEComposer(dwell = 200)

        state = goal.state.states
        sourcePos = ChessPos(goal.source[0], goal.source[1], True)
        targetPos = ChessPos(goal.target[0], goal.target[1], True)
        print('defeat move: {0} -> {1}'.format(sourcePos, targetPos))
        # clear source from state array
        state[sourcePos.to_idx(True)] = False

        result, path_cost, intermediate = pathfinding(sourcePos, targetPos, state, True)

        #remove offset from paths
        #offset is added for defeat slots
        for move in intermediate:
            move.start.x -= 1
            move.end.x -= 1

        for move in result:
            move.x -= 1

        

        print(result)

        path_merged = path_merge_delta(result)

        for move in intermediate:
            gc.MoveCommandQueue(move.start, move.end)

        gc.MoveEffectorAlongPath(path_merged)

        for move in intermediate:
            gc.MoveCommandQueue(move.end, move.start)

        gc.LogCommands()

        #goal = marlin_serial.msg.GcodeGoal(commands=self._cmd)
        # TODO: Fixed neutral position?
        if success:
            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

class PieceMoveSimpleAction:
    _feedback = marlin_motion_planner.msg.PieceMoveSimpleActionFeedback()
    _result = marlin_motion_planner.msg.PieceMoveSimpleActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(name, marlin_motion_planner.msg.PieceMoveSimpleAction,
        execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        r = rospy.Rate(10)
        success = True

        #MoveEffectorTo(goal.target)

        if success:
            #self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)



if __name__ == '__main__':
    rospy.init_node('marlin_motion_planner')
    defServer = PieceDefeatedAction('planner_defeated')
    moveServer = PieceMoveAction('planner_move')
    moveSimpleServer = PieceMoveSimpleAction('planner_move_simple')
    rospy.spin()