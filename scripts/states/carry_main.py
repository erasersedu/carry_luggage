#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import tf2_ros
import math
import numpy as np
import random
import traceback
import time

from states.move_arm import MoveArm
from states.follow_person import FollowPerson
from states.move_base import MoveBase
from states.speech import RobotSay, RobotPlay, RobotYesNo

print("Initializing...")
rospy.sleep(1)

def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure'])

  sm.userdata.pose = [0.0, 0.0, 0.0]

  sm.userdata.start_time = []
  sm.userdata.stop_time = []

  with sm:
	##########
	#START: BASIC FUNCTION
	##########
	@smach.cb_interface(outcomes=['success', 'failure'])
	def start_cb(userdata):
		try:
			rospy.sleep(10)
			print("First state.")

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('START', smach.CBState(start_cb),
				transitions = {'success': 'PLAY1', #'SETPOSE2', #'SETPOSE', #'FOLLOWPERSON', #'MOVEARM',
					       'failure': 'failure'})

	smach.StateMachine.add('PLAY1', RobotPlay(path = "/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav"),
                               transitions={'success': 'Head for', 'failure': 'failure'}) #R2-D2(Start)

	smach.StateMachine.add('Head for', RobotSay(message = "I will go there soon", delay = 8),
                               transitions={'success': 'SETPOSE2', 'failure': 'failure'})

	#Move rel example
	@smach.cb_interface(outcomes=['success','failure'],
			    input_keys=['pose'], output_keys=['pose'])
	def set_pose_cb(userdata):
		try:
			userdata.pose = [-0.683, 0.393, -2.935]

			return 'success'
		except:
			return 'failure'
	smach.StateMachine.add('SETPOSE2', smach.CBState(set_pose_cb),
		           transitions = {'success': 'MOVEBASE2',
		                          'failure': 'failure'})

        smach.StateMachine.add('MOVEBASE2', MoveBase(mode = 'abs'),
                               transitions={'success': 'SAY', 'timeout': 'failure', 'failure': 'failure'}) #MOVEARM


	##Speech##
	smach.StateMachine.add('SAY', RobotSay(message = "Hello, I am Turtlebot.", delay = 3),
                               transitions={'success': 'help?', 'failure': 'failure'})
	smach.StateMachine.add('help?', RobotYesNo(message = "Shall I help you?"),
                               transitions={'yes': 'carry_luggage?', 'no':'PLAY2', 'failure': 'CONFIRMATION'})
	smach.StateMachine.add('carry_luggage?', RobotYesNo(message = "Shall I carry your luggage?"),
                               transitions={'yes': 'MOVEARM', 'no':'New_task', 'failure': 'CONFIRMATION'})
	smach.StateMachine.add('New_task', RobotSay(message = "Sorry.I have no other features yet."),
                               transitions={'success': 'help?', 'failure': 'failure'})
	smach.StateMachine.add('CONFIRMATION', RobotYesNo(message = "Can you hear me?"),
                               transitions={'yes': 'help?', 'no':'CONFIRMATION', 'failure': 'failure'})
	##Speech##


	##ARM##
        smach.StateMachine.add('MOVEARM', MoveArm(target = 'vertical', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 3),
                               transitions={'success': 'MOVEARM2', 'timeout': 'failure', 'failure': 'failure'})

        smach.StateMachine.add('MOVEARM2', MoveArm(target = 'front2', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 5),
                               transitions={'success': 'SAY2', 'timeout': 'failure', 'failure': 'failure'})

	smach.StateMachine.add('SAY2', RobotSay(message = "Please give your luggage to me"),
                               transitions={'success': 'MOVEARM3', 'failure': 'failure'})

        smach.StateMachine.add('MOVEARM3', MoveArm(target = 'front', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 10),
                               transitions={'success': 'MOVEARM4', 'timeout': 'failure', 'failure': 'failure'})

	smach.StateMachine.add('MOVEARM4', MoveArm(target = 'carrying', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 9),
                               transitions={'success': 'FOLLOWPERSON', 'timeout': 'failure', 'failure': 'failure'})
	##ARM##


	##Follow me##
	smach.StateMachine.add('FOLLOWPERSON', FollowPerson(delay = 60),
                               transitions={'success': 'MOVEARM5', 'timeout': 'failure', 'failure': 'failure'})


	##hand over,Return##
	smach.StateMachine.add('MOVEARM5', MoveArm(target = 'front2', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 11),
                               transitions={'success': 'Take', 'timeout': 'failure', 'failure': 'failure'})

	smach.StateMachine.add('Take', RobotSay(message = "Please take your luggage"),
                               transitions={'success': 'MOVEARM6', 'failure': 'failure'})

	smach.StateMachine.add('MOVEARM6', MoveArm(target = 'vertical', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 10),
                               transitions={'success': 'once again?', 'timeout': 'failure', 'failure': 'failure'})

	smach.StateMachine.add('once again?', RobotYesNo(message = "What else do you want me to do?"),
                               transitions={'yes': 'Return', 'no':'mission_complete!', 'failure': 'mission_complete!'})
	
	smach.StateMachine.add('mission_complete!', RobotSay(message = "mission complete!"),
                               transitions={'success': 'mission_complete!_Return', 'failure': 'failure'})
	smach.StateMachine.add('mission_complete!_Return', RobotSay(message = "I will return to the original position.", delay = 7),
                               transitions={'success': 'SETPOSE3', 'failure': 'failure'})
	##hand over,Return##

	##once again##
	smach.StateMachine.add('Return', RobotSay(message = "I'll be back."),
                               transitions={'success': 'SETPOSE_n', 'failure': 'failure'}) #I will be back
	@smach.cb_interface(outcomes=['success','failure'],
			    input_keys=['pose'], output_keys=['pose'])
	def set_pose_n_cb(userdata):
	 	try:
			rospy.sleep(8)
			userdata.pose = [-0.683, 0.393, -2.935] #In front of locker = [-0.149,-3.372,0.753],chair = [-0.683, 0.393, -2.935]

			return 'success'
		except:
			return 'failure'
	smach.StateMachine.add('SETPOSE_n', smach.CBState(set_pose_cb),
		           transitions = {'success': 'MOVEBASE_n',
		                          'failure': 'failure'})
        smach.StateMachine.add('MOVEBASE_n', MoveBase(mode = 'abs'),
                               transitions={'success': 'carry_luggage?', 'timeout': 'failure', 'failure': 'failure'})
	##once again##

	#Mive abs example(Place to return)
	@smach.cb_interface(outcomes=['success','failure'],
			    input_keys=['pose'], output_keys=['pose'])
	def set_pose2_cb(userdata):
		try:
			rospy.sleep(8)
			userdata.pose = [-0.683, 0.393, -2.935] #In front of locker = [-0.149,-3.372,0.753],chair = [-0.683, 0.393, -2.935]

			return 'success'
		except:
			return 'failure'
	smach.StateMachine.add('SETPOSE3', smach.CBState(set_pose2_cb),
		           transitions = {'success': 'MOVEBASE3',
		                          'failure': 'failure'})

        smach.StateMachine.add('MOVEBASE3', MoveBase(mode = 'abs'),
                               transitions={'success': 'PLAY2', 'timeout': 'failure', 'failure': 'failure'})


	smach.StateMachine.add('PLAY2', RobotPlay(path = "/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav"),
                               transitions={'success': 'FINAL', 'failure': 'failure'}) #R2-D2(End)

	@smach.cb_interface(outcomes=['success', 'failure'])
	def final_cb(userdata):
		try:
			print("Last state.")

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('FINAL', smach.CBState(final_cb),
				transitions = {'success': 'success',
					       'failure': 'failure'})
	##########
	#END: BASIC FUNCTION2
	##########

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

  return sm

#######
#######
rospy.init_node('my_carry')

sm = create_sm()

outcome = sm.execute()

if outcome == 'success':
	print("I finished the task.")

else:
	print("Some error occured.")
	rospy.signal_shutdown('failure')
