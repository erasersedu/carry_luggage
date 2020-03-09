#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import rospy
import smach
import traceback
import time

from std_msgs.msg import Bool, String

#Sound libraries
import os, sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

#Global variables
THRESHOLD = 12.0

class RobotSay(smach.State):
	def __init__(self, sentence = "Hello!", delay = 5, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure'])

		self.soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		self.soundhandle.stopAll()

		self.sentence = sentence
		self.delay = delay

	def execute(self, userdata):
		try:
			rate = rospy.Rate(30)

			self.soundhandle.say(self.sentence)
			rospy.sleep(self.delay)

			return 'success'

		except:
			print(traceback.format_exc())

			return 'failure'

class RobotPlay(smach.State):
	def __init__(self, path = [], timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure'])

		self.soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		self.soundhandle.stopAll()

		self.path = path

	def execute(self, userdata):
		try:
			rate = rospy.Rate(30)

			if path and os.path.exists(self.path):
				self.soundhandle.playWave(self.path)
			else:
				self.soundhandle.say('Hello!')
				rospy.sleep(3)

			return 'success'

		except:
			self.soundhandle.say('Hello!')
			rospy.sleep(3)

			print(traceback.format_exc())

			return 'failure'
