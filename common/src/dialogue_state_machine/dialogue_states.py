#!/usr/bin/env python

from __future__ import print_function

import copy

import rospy
import smach
import smach_ros
import threading
from smach import State, StateMachine

from std_msgs.msg import Int32, String


class DialogueSubStateMachine(smach.StateMachine):

    def __init__(self, state_out):

        StateMachine.__init__(self, outcomes=[state_out, 'dialogue_end'])

        with self:

            smach.StateMachine.add(
                'DIALOGUE_BEGIN',
                DialogueBeginState(),
                transitions={
                    'success': 'SPEECH_RECOGNITION',
                    'failure': 'DIALOGUE_BEGIN'
                }
            )
            
            smach.StateMachine.add(
                'SPEECH_RECOGNITION',
                SpeechRecognitionState(),
                transitions={
                    'continue': 'SPEECH_RECOGNITION_WAITING'
                }
            )

            smach.StateMachine.add(
                'SPEECH_RECOGNITION_WAITING',
                SpeechRecognitionWaitingState(),
                transitions={
                    'success': 'DIALOGUE_MANAGEMENT',
                    'failure': 'SPEECH_RECOGNITION_FAILURE'
                }
            )

            smach.StateMachine.add(
                'SPEECH_RECOGNITION_FAILURE',
                SpeechRecognitionFailureState(),
                transitions={
                    'success': 'SPEECH_RECOGNITION',
                    'failure': 'SPEECH_RECOGNITION_FAILURE'
                }
            )

            smach.StateMachine.add(
                'DIALOGUE_MANAGEMENT',
                DialogueManagementState(),
                transitions={
                    'continue': 'DIALOGUE_CONTINUE',
                    'finish': 'DIALOGUE_END'
                }
            )

            smach.StateMachine.add(
                'DIALOGUE_CONTINUE',
                DialogueContinueState(),
                transitions={
                    'success': 'SPEECH_RECOGNITION',
                    'failure': 'DIALOGUE_CONTINUE'
                }
            )

            smach.StateMachine.add(
                'DIALOGUE_END',
                DialogueEndState(),
                transitions={
                    'success': state_out,
                    'failure': 'DIALOGUE_END'
                }
            )


class DialogueBeginState(State):

    def __init__(self):
        State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        #hri_states.Say("Something to start dialogue")
        rospy.loginfo('Executing DialogueBegin State')
        return 'success'


class SpeechRecognitionState(State):

    def __init__(self):
        
        State.__init__(self, outcomes=['continue'])

        rospy.loginfo('Initializing SpeechRecognition State')
        self.publisher = rospy.Publisher(
            '/do_recognition', Int32, queue_size=1
        )

    def execute(self, userdata):

        rospy.loginfo('Executing SpeechRecognition State')
        rospy.sleep(0.1)

        # starts recognition listenning for 5 seconds
        self.publisher.publish(Int32(data=5))

        return 'continue'

class SpeechRecognitionWaitingState(State):

    def __init__(self):
        
        State.__init__(self, outcomes=['success', 'failure'])
        
        #self.mutex = threading.Lock()
        self.speech_recog_status = None
        self.received_msg = False

        self.subsriber = rospy.Subscriber(
            '/recognition_status', String, self.callback
        )

    def callback(self, msg):
        self.received_msg = True
        self.speech_recog_status = msg.data

    def execute(self, userdata):

        rospy.loginfo('Executing SpeechRecognitionWaiting State')
        
        #return 'success'

        while not self.received_msg and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.received_msg = False

        if self.speech_recog_status == 'success':
            return 'success'
            
        return 'failure'

class SpeechRecognitionFailureState(State):

    def __init__(self):
        State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo('Exeuting SpeechRecognitionFailure State')
        return 'success'
        #hri_states.Say("Can you repeat what you said?")

class DialogueManagementState(State):

    def __init__(self):
        State.__init__(self, outcomes=['continue', 'finish'])

    def execute(self,userdata):
        rospy.loginfo('Executing DialogueManagement State')
        return 'finish'

class DialogueContinueState(State):

    def __init__(self):
        State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo('Executing DialogueContinue State')
        return 'success'
        #hri_states.Say("text generated by the dialogue management")

class DialogueEndState(State):

    def __init__(self):
        State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo('Executing DialogueEndState')
        return 'success'
        #hri_states.Say("Something to end conversation based on the outcome")