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
	def __init__(self, message = "Hello!", delay = 5):
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

			if not path == "" and os.path.exists(self.path):
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

class RobotYesNo(smach.State):
	def __init__(self, message = "Hello!", delay = 5):
		smach.State.__init__(self, outcomes=['yes', 'no', 'failure'])

		self.soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		self.soundhandle.stopAll()

		# Subscribe to the recognizer output and set the callback function
		rospy.Subscriber('/lm_data', String, self.talkback_cb)

		self.yes = False
		self.no = False

		self.message = message
		self.delay = delay

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
			rospy.sleep(self.delay)

			self.soundhandle.say('Say yes or no.')
			rospy.sleep(2)

			#Call startstop function
			stop = False
			start_time = time.time()
			while self.yes == False and self.no == False and stop == False:

				if time.time() - start_time > 60:
					stop = True

				rate.sleep()

			if self.yes:
				self.soundhandle.say('You said yes.')
				rospy.sleep(2)
				return 'yes'
			elif self.no:
				self.soundhandle.say('You said no.')
				rospy.sleep(2)
				return 'no'
			else:
				self.soundhandle.say("Sorry, I didn't hear you.")
				rospy.sleep(2)
				return 'failure'

		except:
			self.fp_legs_found = False

			self.fp_enable_leg_finder_pub.publish(False)
			self.fp_start_follow_pub.publish(False)

			print(traceback.format_exc())

			return 'failure'

