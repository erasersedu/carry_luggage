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
	def __init__(self, message = "Hello!", delay = 1):
		smach.State.__init__(self, outcomes=['success', 'failure'])

		self.soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		self.soundhandle.stopAll()

		self.message = message
		self.delay = delay

	def execute(self, userdata):
		try:
			rate = rospy.Rate(30)

			self.soundhandle.say(self.message)
			rospy.sleep(self.delay)

			return 'success'

		except:
			print(traceback.format_exc())

			return 'failure'

class RobotPlay(smach.State):
	def __init__(self, path = ""):
		smach.State.__init__(self, outcomes=['success', 'failure'])

		self.soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		self.soundhandle.stopAll()

		self.path = path

	def execute(self, userdata):
		try:
			rate = rospy.Rate(30)

			if not self.path == "" and os.path.exists(self.path):
				self.soundhandle.playWave(self.path)
				rospy.sleep(1)
			else:
				self.soundhandle.say('Hello!')
				rospy.sleep(1)

			return 'success'

		except:
			self.soundhandle.say('Hello!')
			rospy.sleep(3)

			print(traceback.format_exc())

			return 'failure'

class RobotYesNo(smach.State):
	def __init__(self, message = "Hello!"):
		smach.State.__init__(self, outcomes=['yes', 'no', 'failure'])

		self.soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		self.soundhandle.stopAll()

		# Subscribe to the recognizer output and set the callback function
		rospy.Subscriber('/lm_data', String, self.talkback_cb)

		self.yes = False
		self.no = False

		self.message = message

	def talkback_cb(self, msg):
		# Print the recognized words on the screen
		rospy.loginfo(msg.data)

		self.yes = False
		self.no = False

		if msg.data.find('ROBOT-YES')>-1:
			self.yes = True

		elif msg.data.find('ROBOT-NO')>-1:
			self.no = True

	def execute(self, userdata):
		try:
			rate = rospy.Rate(30)

			self.soundhandle.say(self.message)

			self.soundhandle.say('Say robot yes or robot no.')
			rospy.sleep(1)

			#Call startstop function
			stop = False
			start_time = time.time()
			while self.yes == False and self.no == False and stop == False:

				if time.time() - start_time > 60:
					stop = True

				rate.sleep()

			if self.yes:
				self.soundhandle.say('You said yes.')
				self.yes = False
				self.no = False

				rospy.sleep(1)
				return 'yes'
			elif self.no:
				self.soundhandle.say('You said no.')
				self.yes = False
				self.no = False

				rospy.sleep(1)
				return 'no'
			else:
				self.soundhandle.say("Sorry, I didn't hear you.")
				self.yes = False
				self.no = False

				rospy.sleep(1)
				return 'failure'

		except:
			self.yes = False
			self.no = False

			self.soundhandle.say("Sorry, I didn't hear you.")
			rospy.sleep(1)

			print(traceback.format_exc())

			return 'failure'

