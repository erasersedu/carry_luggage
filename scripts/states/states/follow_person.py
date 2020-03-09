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

class FollowPerson(smach.State):
	def __init__(self, delay = 60, timeout = None):
		smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
					   input_keys = ['start_time', 'stop_time'])

		#rospy.init_node('follow_person', anonymous = True)

		self.soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		self.soundhandle.stopAll()

		# Announce that we are ready for input
		#self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")

		# Subscribe to the recognizer output and set the callback function
		rospy.Subscriber('/lm_data', String, self.talkback_cb)

		#Leg finder
		self.fp_enable_leg_finder_pub = rospy.Publisher('/leg_finder/enable', Bool)
		self.fp_start_follow_pub = rospy.Publisher('/human_following/start_follow', Bool)

		self.fp_legs_found_sub = rospy.Subscriber('/leg_finder/legs_found', Bool, self._fp_legs_found_cb)

		#TODO: Call STARTSTOP publisher/subscriber
		#self.fp_wrist_wrench_sub = rospy.Subscriber('/wrist_wrench/raw', WrenchStamped, self._wrist_wrench_cb)

		self.fp_legs_found = False

		self.start = False
		self.stop = False

		self.delay = delay

	def _fp_legs_found_cb(self, msg):
		try:
		    if msg.data == True and msg.data != self.fp_legs_found:
			    self.fp_legs_found = True

			    self.soundhandle.say('I found you!')
			    rospy.sleep(1)
			    print("Legs found")
			    rospy.loginfo('Legs found')

		    elif msg.data == False and msg.data != self.fp_legs_found:
			    self.fp_legs_found = False

			    self.soundhandle.say('Sorry, I lost you! Please come where I can see you.')
			    rospy.sleep(1)

			    rospy.loginfo('Legs lost')
		except:
		    self.fp_legs_found = False

	def talkback_cb(self, msg):
		# Print the recognized words on the screen
		#msg.data=msg.data.lower()
		rospy.loginfo(msg.data)

		self.start = False
		self.stop = False

		if msg.data.find('FOLLOW-ME')>-1:
			#self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			self.start = True

		elif msg.data.find('STOP-FOLLOW')>-1:
			#self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			self.stop = True

		#else: rospy.sleep(3)

	def execute(self, userdata):
		try:
			rate = rospy.Rate(30)

			self.fp_enable_leg_finder_pub.publish(False)
			self.fp_start_follow_pub.publish(False)

			self.soundhandle.say('Say Follow Me to start following you.')
			rospy.sleep(1)

			#Call startstop function
			while self.start == False:
				rate.sleep()

			self.soundhandle.say('Now I will find you.')
			rospy.sleep(1)

			self.fp_enable_leg_finder_pub.publish(True)
			self.fp_start_follow_pub.publish(True)

			#start_time = time.time()
			while self.stop == False:
				if self.fp_legs_found == False:
					self.fp_start_follow_pub.publish(False)
					while self.fp_legs_found == False:
						rate.sleep()

					self.fp_start_follow_pub.publish(True)

				#if time.time() - start_time > self.delay:
				#	self.startstop = True

				rate.sleep()

			self.fp_start_follow_pub.publish(False)
			self.fp_enable_leg_finder_pub.publish(False)

			self.soundhandle.say('OK, I will stop following you.')
			rospy.sleep(1)

			return 'success'

			#return 'timeout'
		except:
			self.fp_legs_found = False

			self.fp_enable_leg_finder_pub.publish(False)
			self.fp_start_follow_pub.publish(False)

			print(traceback.format_exc())

			return 'failure'

