#! /usr/bin/env python

import actionlib
import marlin_motion_planner.msg
import marlin_serial.msg
import rospy

class GCODEComposer:
    def __init__(self, dwell = 200):
        self._cmd_buffer = []
        self._dwell = dwell
        
    def ApplyMarlinOffsetScaled(self, target):
        # TODO: load offsets from config
        offsets = [0.0, 25.0]
        retVal = []
        retVal.append(target[0] * 100.0 + offsets[0])
        retVal.append(target[1] * 100.0 + offsets[1])
        return retVal

    def ActivateMagnet(self, state = False):
        if state == True:
            self._cmd_buffer.append('M106')
            if self._dwell > 0:
                self._cmd_buffer.append('G4 P{0}'.format(self._dwell))
        else:
            self._cmd_buffer.append('M107')
            

    def MoveEffectorTo(self, target):
        marlin_target = ApplyMarlinOffsetScaled(target)
        cmd = 'G1 X{0} Y{1}'.format(marlin_target[0], marlin_target[1]) # feedrate?
        self._cmd_buffer.append(cmd)

    def MoveEffectorAlongPath(self, path):
        cmd = []
        for segment in path:
            self._cmd_buffer.append(MoveEffectorTo(segment))

    def LogCommands(self):
        rospy.loginfo('Marlin commands: {0}'.format(len(self._cmd_buffer)))
        for (idx, cmd) in enumerate(self._cmd_buffer):
            rospy.loginfo('{0} : {1}'.format(idx, cmd))
    
    def GetBuffer(self):
        return self._cmd_buffer

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
        r = rospy.Rate(10)
        success = True

        gc = GCODEComposer(dwell = 200)

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
        r = rospy.Rate(10)
        success = True

        # calculate path
        # move to source
        # activate magnet
        # move along the path
        # deactivate magnet
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