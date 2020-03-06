#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
import numpy as np
import traceback
import time

#navigation libraries
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class MoveBase(smach.State):
	def __init__(self, pose = [0.0, 0.0, 0.0], mode = 'abs', timeout=None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['pose', 'start_time', 'stop_time'])

		# Subscribe to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server...")

		# Wait for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(120))
		rospy.loginfo("Connected to move base server")

		self.mode = mode
		self.goal_sent = False

	def execute(self, userdata):
		try:
			quaternion = quaternion_from_euler(0.0, 0.0, userdata.pose[2])
			location = Pose(Point(userdata.pose[0], userdata.pose[1], 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

			goal = MoveBaseGoal()

			if self.mode == 'abs':
				goal.target_pose.header.frame_id = 'map'
			elif self.mode == 'rel':
				goal.target_pose.header.frame_id = 'base_link'
			else:
				goal.target_pose.header.frame_id = 'base_link'

			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose = location

			# Start moving
			self.goal_sent = True
			self.move_base.send_goal(goal)

			# Allow TurtleBot up to 300 seconds to complete task
			success = self.move_base.wait_for_result(rospy.Duration(300)) 

			state = self.move_base.get_state()
			result = False

			if userdata.start_time and userdata.stop_time:
				interval = time.time() - userdata.start_time
				if interval > userdata.stop_time:
					return 'timeout'

			if success and state == GoalStatus.SUCCEEDED:
			    self.goal_sent = False
			    return 'success'
			else:
			    self.move_base.cancel_goal()
			    return 'failure'

		except:
			if self.goal_sent:
			    self.move_base.cancel_goal()

			print(traceback.format_exc())
			return 'failure'

