#!/usr/bin/env python

import string
import rospy
import smach
import smach_ros
from chessboard_uci.msg import ucicommandGoal
from chessboard_uci.msg import ucicommandAction
from marlin_serial.msg import MoveActionAction
from marlin_serial.msg import MoveActionGoal
from marlin_serial.msg import ResetAction
from marlin_serial.msg import ResetGoal
from smach_ros import SimpleActionState
from smach import Sequence
from smach import Iterator
from actionlib_msgs.msg import GoalStatus


#oof
# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'

def main():
    uci_topic = 'chessboard_uci_node'
    chesspiece_topic = 'chessboard_chesspieces_node'

    rospy.init_node('chessboard_sm_node')


    sm = smach.StateMachine(outcomes=['finished', 'succeeded', 'preempted', 'aborted'])
    sm.userdata.target = []

    sis = smach_ros.IntrospectionServer('chessboard_sm_sis', sm, '/SM_ROOT')
    sis.start()

    with sm:

        sq_init = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
        connector_outcome='succeeded')

        #Stockfish init
        with sq_init:
            rospy.loginfo('Stockfish init')

            Sequence.add('PIECE_RESET', SimpleActionState('chessboard_chesspieces_reset', ResetAction))
            Sequence.add('UCI_INIT', SimpleActionState(uci_topic, ucicommandAction, 
            goal=ucicommandGoal(cmd=ucicommandGoal.CMD_INIT)))
            Sequence.add('UCI_NEWGAME', SimpleActionState(uci_topic, ucicommandAction, 
            goal=ucicommandGoal(cmd=ucicommandGoal.CMD_NEWGAME)))

        smach.StateMachine.add('STOCKFISH_SETUP', sq_init, transitions={'succeeded' : 'UCI_ITER'})

        #Test code

        tutorial_it = Iterator(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = [],
                               it = lambda: range(0, 2),
                               output_keys = [],
                               it_label = 'index',
                               exhausted_outcome = 'succeeded')
        with tutorial_it:
            container_sm = smach.StateMachine(outcomes = ['finished','succeeded','preempted','aborted','continue'])
            with container_sm:

                def calc_result_cb(userdata, status, result):
                    if status == GoalStatus.SUCCEEDED:
                        result_coords = list(result.result.data)
                        print(result.result.data)
                        new_target = []
                        # convert chess notation to indices
                        # e.g. b8c6 -> 6,7,5,5
                        new_target.append(7 - int(string.lowercase.index(result_coords[0])))
                        new_target.append(int(result_coords[1]) -1)
                        new_target.append(7 - string.lowercase.index(result_coords[2]))
                        new_target.append(int(result_coords[3]) - 1)
                        #print(new_target)
                        userdata.target = new_target
                        return 'succeeded'

                smach.StateMachine.add('UCI_CALCMOVE', SimpleActionState(uci_topic, ucicommandAction, 
                goal=ucicommandGoal(cmd=ucicommandGoal.CMD_CALC_MOVE),
                result_cb=calc_result_cb,
                output_keys=['target']),
                transitions={'succeeded' : 'PIECE_MOVE'},
                remapping={'target':'user_data_target'})


                smach.StateMachine.add('PIECE_MOVE', SimpleActionState(chesspiece_topic, MoveActionAction, 
                goal_slots=['target']), 
                transitions={'succeeded' : 'UCI_MOVE'},
                remapping={'target':'user_data_target'})

                smach.StateMachine.add('UCI_MOVE', SimpleActionState(uci_topic, ucicommandAction, 
                goal=ucicommandGoal(cmd=ucicommandGoal.CMD_MOVE)), 
                transitions={'succeeded' : 'finished'})
            

            Iterator.set_contained_state('CONTAINER_STATE', 
                                        container_sm, 
                                        loop_outcomes=['finished'])

        smach.StateMachine.add('UCI_ITER',tutorial_it,
                     {'succeeded':'succeeded',
                      'aborted':'aborted'})


        # uci_node -> CMD_CALC_MOVE
        # uci_node -> CMD_MOVE
        #get new target position
        #movement
        #set position in uci
    
    outcome = sm.execute()

    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
