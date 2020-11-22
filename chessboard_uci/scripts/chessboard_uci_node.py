#! /usr/bin/env python

import actionlib
import chessboard_uci.msg
import rospy
import std_msgs
from chessboard_uci.msg import ucicommandGoal


class UCIAction(object):
    _feedback = chessboard_uci.msg.ucicommandFeedback()
    _result = chessboard_uci.msg.ucicommandResult()
    _cur_goal = chessboard_uci.msg.ucicommandGoal()
    _success = False
    _move_history = []
    

    def stockfish_cb(self, data):
        rospy.logdebug(rospy.get_caller_id() + ' got data %s', data.data)
        if self._cur_goal.cmd == -1:
            rospy.loginfo('No goal is set')
            return

        # remove endline
        data.data = data.data.rstrip()
        #rospy.loginfo('CMD: %u', self._cur_goal.cmd)
        if self._cur_goal.cmd == ucicommandGoal.CMD_INIT:
            if data.data == 'uciok':
                self._stockfish_pub.publish('isready\n')
                return
            elif data.data == 'readyok':
                rospy.loginfo('CMD_INIT done')
                self._success = True
                return
        elif self._cur_goal.cmd == ucicommandGoal.CMD_CALC_MOVE:
            if data.data.startswith('bestmove '):
                nextmove = data.data[9:13]
                rospy.loginfo('best move: ' + nextmove)
                self._move_history.append(nextmove) #debug

                self._result.result.data = nextmove
                self._success = True
                return
                # ponder?
        #elif self._cur_goal.cmd == ucicommandGoal.CMD_MOVE:
        

        self._feedback.feedback = data
        self._as.publish_feedback(self._feedback)




    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                chessboard_uci.msg.ucicommandAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # init UCI subscriber / publisher
        rospy.Subscriber('stockfish/stdout',
                         std_msgs.msg.String, self.stockfish_cb)
        self._stockfish_pub = rospy.Publisher('stockfish/stdin', std_msgs.msg.String, queue_size=10)

    def execute_cb(self, goal):
        r = rospy.Rate(1)
        self._success = False

        rospy.loginfo('executing action %u', goal.cmd)
        self._cur_goal = goal


        if goal.cmd == chessboard_uci.msg.ucicommandGoal.CMD_INIT:
            rospy.loginfo("CMD_INIT")
            self._stockfish_pub.publish('uci\n')

        elif goal.cmd == ucicommandGoal.CMD_NEWGAME:
            # clead movehistory
            self._stockfish_pub.publish('ucinewgame\n')
            self._success = True

        elif goal.cmd == ucicommandGoal.CMD_CALC_MOVE:
            self._stockfish_pub.publish('go\n')
            self._success = True

        elif goal.cmd == ucicommandGoal.CMD_MOVE:
            # convert coords
            if len(self._move_history) == 0:
                rospy.loginfo('no positions to send')
                self._success = True
            else:
                uci_position_str = 'position startpos moves '
                uci_position_str += ' '.join(self._move_history) + '\n'
                rospy.loginfo(uci_position_str)
                self._stockfish_pub.publish(uci_position_str)
                self._success = True

        

        while self._success is False:
            r.sleep()

        if self._success is True:
            self._as.set_succeeded(self._result)

        




if __name__ == '__main__':
    rospy.init_node('chessboard_uci_node')
    rospy.loginfo('chessboard_uci init')
    server = UCIAction(rospy.get_name())
    rospy.spin()
