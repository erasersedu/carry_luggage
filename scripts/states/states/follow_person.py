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
		self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")

		# Subscribe to the recognizer output and set the callback function
		rospy.Subscriber('/lm_data', String, self.talkback)

		#Leg finder
		self.fp_enable_leg_finder_pub = rospy.Publisher('/leg_finder/enable', Bool)
		self.fp_start_follow_pub = rospy.Publisher('/human_following/start_follow', Bool)

		self.fp_legs_found_sub = rospy.Subscriber('/leg_finder/legs_found', Bool, self._fp_legs_found_cb)

		#TODO: Call STARTSTOP publisher/subscriber
		#self.fp_wrist_wrench_sub = rospy.Subscriber('/wrist_wrench/raw', WrenchStamped, self._wrist_wrench_cb)

		self.fp_legs_found = False

		self.startstop = False

		self.delay = delay

	def _fp_legs_found_cb(self, msg):
		try:
		    if msg.data == True and msg.data != self.fp_legs_found:
			    self.fp_legs_found = True

			    self.soundhandle.say('I found you!')
			    rospy.sleep(3)
			    print("Legs found")
			    rospy.loginfo('Legs found')

		    elif msg.data == False and msg.data != self.fp_legs_found:
			    self.fp_legs_found = False

			    self.soundhandle.say('Sorry, I lost you! Please come where I can see you.')
			    rospy.sleep(6)

			    rospy.loginfo('Legs lost')
		except:
		    self.fp_legs_found = False

	#TODO: STARTSTOP callback (voice?)
	#def _wrist_wrench_cb(self, msg):
	#	try:
	#		self.startstop = False
	#		current_value = msg.wrench.force.x
	#		if THRESHOLD > 0.:
	#		    if current_value > THRESHOLD:
	#			self.startstop = True
	#		else:
	#		    if current_value < -THRESHOLD:
	#			self.startstop = True
	#	except:
	#	    self.startstop = False

	def talkback(self, msg):
		# Print the recognized words on the screen
		#msg.data=msg.data.lower()
		rospy.loginfo(msg.data)
        
		if msg.data.find('INTRODUCE-YOURSELF')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("I heard you want me to introduce myself. I am PartyBot. I am a party robot to serve you and have fun.")
			#rospy.sleep(10) 
		elif msg.data.find('HOW-OLD-ARE-YOU')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("I heard you ask about my age. I am five years old.")
			#rospy.sleep(5) 
		elif msg.data.find('FOLLOW-ME')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("OK. I will start follow you.")
			#rospy.sleep(5)
			self.control_follow(1)
		elif msg.data.find('STOP-FOLLOW')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("OK. I will stop follow you.")
			#rospy.sleep(5)
			self.control_follow(0)
		elif msg.data.find('ARE-YOU-FROM')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("I heard you ask about my hometown. I am from China.")
			#rospy.sleep(5)
		elif msg.data.find('CAN-YOU-DO')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("I heard you ask me what can I do? I am a home robot. I am good at singing and dancing. I tell funny jokes and I take great photos of people")
			#rospy.sleep(5)
		elif msg.data.find('TELL-A-FUNNY-JOKE')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("You want to hear a joke? What is orange and sounds like a parrot? Erm, It is a carrot. Ha ha ha")
			#rospy.sleep(8)
		elif msg.data.find('SING-AND-DANCE')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			self.soundhandle.say("You want me to sing and dance? sure. let me show you")
			#rospy.sleep(5)
			self.dance_arm.publish('dance arm')      	
			self.soundhandle.playWave("~/erasersedu_ws/src/carry_luggage/sounds/swtheme.wav", blocking=False)
			#rospy.sleep(1) 
			# Dancing
			# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.
			# let's go forward at 0.2 m/s
			move_cmd = Twist()
			move_cmd.linear.x = 0.2
			# by default angular.z is 0 so setting this isn't required
			#let's turn at 45 deg/s
			turn_cmd = Twist()
			turn_cmd.linear.x = 0
			turn_cmd.angular.z = radians(90); #45 deg/s in radians/s
			turn_cmd2 = Twist()
			turn_cmd2.linear.x = 0
			turn_cmd2.angular.z = radians(-90); #45 deg/s in radians/s
			# turn 90 degrees
			rospy.loginfo("Turning")
			for x in range(0,5):
				self.cmd_vel.publish(turn_cmd)
				# r.sleep()
				rospy.sleep(1)
				self.cmd_vel.publish(turn_cmd2)
				rospy.sleep(1) 
			rospy.sleep(6)
		elif msg.data.find('TAKE-A-PHOTO')>-1:
			self.soundhandle.playWave("/home/roboworks/erasersedu_ws/src/carry_luggage/sounds/R2D2a.wav")
			#rospy.sleep(1)
			#call('rosrun image_view image_view image:=/camera_top/rgb/image_raw', shell=False)
			#rospy.sleep(1)
			self.soundhandle.say("You want to take a photo? Ok, get ready. One, two, three, say cheese")
			self.take_photo.publish('take photo')
			#call('rosrun rchomeedu_vision take_photo.py', shell=True)
			#rospy.sleep(6)
			#call('rosrun image_view image_saver image:=/camera_top/rgb/image_raw _save_all_image:=false _filename_format:=foo.jpg __name:=image_saver', shell=True)
			#call('rosservice call /image_saver/save', shell=True)
			#rospy.sleep(6)
		#else: self.soundhandle.say("Sorry, I cannot hear you clearly. Please say again.")
		else: rospy.sleep(3)

	def execute(self, userdata):
		try:
			rate = rospy.Rate(30)

			self.fp_enable_leg_finder_pub.publish(False)
			self.fp_start_follow_pub.publish(False)

			self.soundhandle.say('I will start following you.')
			rospy.sleep(3)

			#TODO: Call startstop function
			#while self.startstop == False:
			#	rate.sleep()

			self.soundhandle.say('Now I will find you.')
			rospy.sleep(3)

			self.fp_enable_leg_finder_pub.publish(True)
			self.fp_start_follow_pub.publish(True)

			start_time = time.time()
			while self.startstop == False:
				if self.fp_legs_found == False:
					self.fp_start_follow_pub.publish(False)
					while self.fp_legs_found == False:
						rate.sleep()

					self.fp_start_follow_pub.publish(True)

				#TODO: Stand-alone callback from subscriber 
				if time.time() - start_time > self.delay:
					self.startstop = True

				rate.sleep()

			self.startstop = False
			self.fp_legs_found = False

			self.fp_enable_leg_finder_pub.publish(False)
			self.fp_start_follow_pub.publish(False)

			self.soundhandle.say('OK, I will stop following you.')

			return 'success'

			#return 'timeout'
		except:
			self.fp_legs_found = False

			self.fp_enable_leg_finder_pub.publish(False)
			self.fp_start_follow_pub.publish(False)

			print(traceback.format_exc())

			return 'failure'

