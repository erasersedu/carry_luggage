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
				transitions = {'success': 'SAY', #'SETPOSE2', #'SETPOSE', #'FOLLOWPERSON', #'MOVEARM',
					       'failure': 'failure'})

        smach.StateMachine.add('FOLLOWPERSON', FollowPerson(delay = 60),
                               transitions={'success': 'FINAL', 'timeout': 'failure', 'failure': 'failure'})

        smach.StateMachine.add('SAY', RobotSay(message = "Hello, I am Turtlebot.", delay = 2),
                               transitions={'success': 'PLAY', 'failure': 'failure'})

        smach.StateMachine.add('PLAY', RobotPlay(path = "/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav"),
                               transitions={'success': 'CONFIRMATION', 'failure': 'failure'})

        smach.StateMachine.add('CONFIRMATION', RobotYesNo(message = "Can you hear me?"),
                               transitions={'yes': 'CONFIRMATION', 'no':'CONFIRMATION', 'failure': 'failure'})

	#Move rel example
	@smach.cb_interface(outcomes=['success','failure'],
			    input_keys=['pose'], output_keys=['pose'])
	def set_pose_cb(userdata):
		try:
		    userdata.pose = [0.2, 0.2, 0.76]

		    return 'success'
		except:
		    return 'failure'
	smach.StateMachine.add('SETPOSE', smach.CBState(set_pose_cb),
		           transitions = {'success': 'MOVEBASE',
		                          'failure': 'failure'})

        smach.StateMachine.add('MOVEBASE', MoveBase(mode = 'rel'),
                               transitions={'success': 'FINAL', 'timeout': 'failure', 'failure': 'failure'})
	#####

	#Move abs example
	@smach.cb_interface(outcomes=['success','failure'],
			    input_keys=['pose'], output_keys=['pose'])
	def set_pose_cb(userdata):
		try:
		    userdata.pose = [0.586, 0.978, 1.826]

		    return 'success'
		except:
		    return 'failure'
	smach.StateMachine.add('SETPOSE2', smach.CBState(set_pose_cb),
		           transitions = {'success': 'MOVEBASE2',
		                          'failure': 'failure'})

        smach.StateMachine.add('MOVEBASE2', MoveBase(mode = 'abs'),
                               transitions={'success': 'FINAL', 'timeout': 'failure', 'failure': 'failure'})
	#####

        smach.StateMachine.add('MOVEARM', MoveArm(target = 'vertical', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 10),
                               transitions={'success': 'MOVEARM2', 'timeout': 'failure', 'failure': 'failure'})

        smach.StateMachine.add('MOVEARM2', MoveArm(target = 'front2', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 10),
                               transitions={'success': 'MOVEARM3', 'timeout': 'failure', 'failure': 'failure'})

        smach.StateMachine.add('MOVEARM3', MoveArm(target = 'front', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 5),
                               transitions={'success': 'FINAL', 'timeout': 'failure', 'failure': 'failure'})

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
