#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
from std_msgs.msg import Float64
import numpy as np
import traceback
import time

class MoveArm(smach.State):
	def __init__(self, target = 'other', pose = [0.0, 0.0, 0.0, 0.0, 0.0], delay = 5, timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['start_time', 'stop_time'])

		# publish command message to joints/servos of arm
	    	self.joint1 = rospy.Publisher('/waist_controller/command',Float64,queue_size=1)
		self.joint2 = rospy.Publisher('/shoulder_controller/command',Float64,queue_size=1)
	    	self.joint3 = rospy.Publisher('/elbow_controller/command',Float64,queue_size=1)
	    	self.joint4 = rospy.Publisher('/wrist_controller/command',Float64,queue_size=1)
		self.joint5 = rospy.Publisher('/hand_controller/command',Float64,queue_size=1)

		self.pos1 = Float64()
	    	self.pos2 = Float64()
	    	self.pos3 = Float64()
	    	self.pos4 = Float64()
	    	self.pos5 = Float64()

		self.delay = delay

		if target == 'vertical':
			self.pos1 = 0.0
		    	self.pos2 = 0.0
		    	self.pos3 = 0.0
		    	self.pos4 = 0.0
		    	self.pos5 = 0.0

		elif target == 'front':
			self.pos1 = 0.0
		    	self.pos2 = 0.3
		    	self.pos3 = 0.3
		    	self.pos4 = -0.3
		    	self.pos5 = 0.0

		elif target == 'left':
			self.pos1 = 0.0
		    	self.pos2 = 0.0
		    	self.pos3 = 0.0
		    	self.pos4 = 0.0
		    	self.pos5 = 0.0

		elif target == 'right':
			self.pos1 = 0.0
		    	self.pos2 = 0.0
		    	self.pos3 = 0.0
		    	self.pos4 = 0.0
		    	self.pos5 = 0.0

		elif target == 'other':
			if len(pose) == 5:
				self.pos1 = pose[0]
			    	self.pos2 = pose[1]
			    	self.pos3 = pose[2]
			    	self.pos4 = pose[3]
			    	self.pos5 = pose[4]
			else:
				self.pos1 = 0.0
			    	self.pos2 = 0.0
			    	self.pos3 = 0.0
			    	self.pos4 = 0.0
			    	self.pos5 = 0.0

		else:
			self.pos1 = 0.0
		    	self.pos2 = 0.0
		    	self.pos3 = 0.0
		    	self.pos4 = 0.0
		    	self.pos5 = 0.0

	def execute(self, userdata):
		try:
			start_time = time.time()
			while time.time() - start_time < self.delay: 
				self.joint1.publish(self.pos1)
				self.joint2.publish(self.pos2)
				self.joint3.publish(self.pos3)
				self.joint4.publish(self.pos4)
				self.joint5.publish(self.pos5)

				rospy.Rate(10).sleep()

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			return 'success'

		except:
			print(traceback.format_exc())
			return 'failure'

