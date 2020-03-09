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
				transitions = {'success': 'SETPOSE2', #'SETPOSE', #'FOLLOWPERSON', #'MOVEARM',
					       'failure': 'failure'})

        smach.StateMachine.add('FOLLOWPERSON', FollowPerson(delay = 60),
                               transitions={'success': 'FINAL', 'timeout': 'failure', 'failure': 'failure'})

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

	#Move abs example
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
                               transitions={'success': 'MOVEARM', 'timeout': 'failure', 'failure': 'failure'})

	##ARM##
        smach.StateMachine.add('MOVEARM', MoveArm(target = 'vertical', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 9),
                               transitions={'success': 'MOVEARM2', 'timeout': 'failure', 'failure': 'failure'})

        smach.StateMachine.add('MOVEARM2', MoveArm(target = 'front2', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 9),
                               transitions={'success': 'MOVEARM3', 'timeout': 'failure', 'failure': 'failure'})

        smach.StateMachine.add('MOVEARM3', MoveArm(target = 'front', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 3),
                               transitions={'success': 'MOVEARM4', 'timeout': 'failure', 'failure': 'failure'})

	smach.StateMachine.add('MOVEARM4', MoveArm(target = 'carrying', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 9),
                               transitions={'success': 'SETPOSE3', 'timeout': 'failure', 'failure': 'failure'})
	##ARM##

	#Mive abs example(Place to return)
	@smach.cb_interface(outcomes=['success','failure'],
			    input_keys=['pose'], output_keys=['pose'])
	def set_pose2_cb(userdata):
		try:
			rospy.sleep(8)
			userdata.pose = [-0.149, -3.372, 0.753]

			return 'success'
		except:
			return 'failure'
	smach.StateMachine.add('SETPOSE3', smach.CBState(set_pose2_cb),
		           transitions = {'success': 'MOVEBASE3',
		                          'failure': 'failure'})

        smach.StateMachine.add('MOVEBASE3', MoveBase(mode = 'abs'),
                               transitions={'success': 'MOVEARM5', 'timeout': 'failure', 'failure': 'failure'})

	##ARM##
	smach.StateMachine.add('MOVEARM5', MoveArm(target = 'front2', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 10),
                               transitions={'success': 'MOVEARM6', 'timeout': 'failure', 'failure': 'failure'})

	smach.StateMachine.add('MOVEARM6', MoveArm(target = 'vertical', pose=[0.0, 0.0, 0.0, 0.0, 0.0], delay = 10),
                               transitions={'success': 'FINAL', 'timeout': 'failure', 'failure': 'failure'})
	##ARM##

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
