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

print("Initializing...")
rospy.sleep(1)

def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure'])

  sm.userdata.start_time = []
  sm.userdata.stop_time = []

  with sm:
	##########
	#START: BASIC FUNCTION
	##########
	@smach.cb_interface(outcomes=['success', 'failure'])
	def start_cb(userdata):
		try:
			print("First state.")

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('START', smach.CBState(start_cb),
				transitions = {'success': 'MOVEARM',
					       'failure': 'failure'})

        smach.StateMachine.add('MOVEARM', MoveArm(target = 'front'),
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
	#END: BASIC FUNCTION
	##########

  return sm

rospy.init_node('my_carry')

sm = create_sm()
outcome = sm.execute()

if outcome == 'success':
	print("I finished the task.")

else:
	print("Some error occured.")
	rospy.signal_shutdown('failure')
