#!/usr/bin/env python

from __future__ import print_function

import copy

import rospy
import smach
import smach_ros
import threading

from dialogue_state_machine.dialogue_states import DialogueSubStateMachine


class GreetingState(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['continue'])

    def execute(self, userdata):

        rospy.loginfo('Executing Greeting State')
        return 'continue'


def main():

    rospy.init_node('smach_example_state_machine')
    rospy.sleep(0.5)

    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS'])

    with sm:

        smach.StateMachine.add(
            'GREETING',
            GreetingState(),
            transitions={
                'continue': 'DIALOGUE_SUB_STATE_MACHINE'
            }
        )

        dialogue_sub_sm = DialogueSubStateMachine(
            state_out='dialogue_end'
        )

        smach.StateMachine.add(
            'DIALOGUE_SUB_STATE_MACHINE',
            dialogue_sub_sm,
            transitions={
                'dialogue_end': 'OVERALL_SUCCESS'
            }
        )

    # Create and start the introspection server (Smach viewer)
    sis = smach_ros.IntrospectionServer('dialogue_sub_sm_viewer', sm, '/DIALOGUE_SM')
    sis.start()

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c to stop the application
    rospy.spin()

    rospy.logwarn("ctrl + c detected!!! preempting smach execution")

    sis.stop()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()

if __name__ == '__main__':

    main()
