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

print("Initializing...")
rospy.sleep(3)

def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure'])

  with sm:
	##########
	#START: BASIC FUNCTION
	##########
	@smach.cb_interface(outcomes=['success', 'failure'])
	def start_cb(userdata):
		try:
			print("First state.")
			rospy.sleep(2)

			return 'success'
		except:
			return 'failure'

	smach.StateMachine.add('START', smach.CBState(start_cb),
				transitions = {'success': 'FINAL',
					       'failure': 'failure'})

	@smach.cb_interface(outcomes=['success', 'failure'])
	def final_cb(userdata):
		try:
			print("Last state.")
			rospy.sleep(2)

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

sm = create_sm()
outcome = sm.execute()

if outcome == 'success':
	rospy.loginfo('I finished the task.')

else:
	rospy.signal_shutdown('Some error occured.')
