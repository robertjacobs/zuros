#!/usr/bin/env python
#!/usr/bin/env python
# Copyright (c) 2013-2014 ZUYD Research
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of the Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author Robert Jacobs/info@rjpjacobs.nl

"""
This program will turn the robot into an assisting robot.
Think of a user which has problems carrying things around the house. Maybe this user had problems with his hands or just cannot carry object for what ever cause.

The first task is to check if the user opened the fridge door (zwave sensor on the fridge door). If the user did, the robot heads to the user to try and assist him. The user can now place a drink in the special holder on the front of the robot. In this holder there is a switch (zwave network switch). When the user places a bottle in the holder, the bottle presses against the switch. This way the robot knows when there is drink placed. 
If the user then heads back to the large sofa or the small sofa and sits on it, the pressure pads (zwave enabled) detect a pressure (the user sitting on it) and then the robot knows where to head. Based on the user location the robot either drives to the small sofa or the large sofa.
While at one of the two sofas the robot waits for the user to remove the bottle from the holder. As soon as the user did the robot heads back to the charging station location.
"""

import roslib
import rospy
import time
import MySQLdb

roslib.load_manifest('zuros_demo')

from include.database import SensorsInDatabase
from zuros_command_to_robot_sender import CommandToRobotSender
from random import randrange

## Demo 2 class
class Demo2(object):
	## init function
	def __init__(self):
		rospy.init_node('demo2_node', anonymous=False)
		rospy.on_shutdown(self.cleanup)
		# Get the sensor information from the database, based on ID
		self.fridge_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:6")
		self.tray_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:26")
		self.large_sofa_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:25")
		self.small_sofa_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:21")

		# Set initial values
		self.fridge_opened = False
		self.user_sat_down = False
		self.drink_placed = False
		self.ctrs = CommandToRobotSender()

	## As soon as we stop the script (or there is a other reason for the scipt to stop) it is best to stop the robot
	def cleanup(self):
		# stop the robot
		rospy.loginfo("Stopping now..")
		self.ctrs.move("base","stop", False)	

	## The run function
	def run(self):
		# Get the current values of the sensors
		fridge_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:6")
		tray_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:26")
		large_sofa_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:25")
		small_sofa_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:21")

		# Fridge opened?
		if(fridge_value != self.fridge_value and self.fridge_opened == False):
			rospy.loginfo("Fridge opened, heading there now..")
			self.ctrs.move("base","kitchen_fridge", False)
			self.fridge_opened = True
			self.fridge_value = fridge_value

		# Fridge has been opened and something has been placed in the holder?
		if(tray_value != self.tray_value and self.user_sat_down == False):
			rospy.loginfo("User placed drink")
			self.drink_placed = True
			self.tray_value = tray_value

 		# Fridge was opened, tray is full and the user sits. Time to bring the goodies!
		if(self.fridge_opened == True and self.drink_placed == True and self.user_sat_down == False):
      # Both seats occupied?
			if(large_sofa_value != self.large_sofa_value and small_sofa_value != self.small_sofa_value):
				self.user_sat_down = True
				self.large_sofa_value = large_sofa_value
				self.small_sofa_value = small_sofa_value
				rospy.loginfo("Both pads occupied, choosing where to go..")
				# Choose randomly where to go
				rand = randrange(0, 10)
				if(rand <= 5):
					rospy.loginfo("Going to large sofa..")
					self.ctrs.move("base","large_sofa_offer", False)
				elif(rand > 5):
					rospy.loginfo("Going to small sofa..")
					self.ctrs.move("base","small_sofa_offer", False)
      
			# Not both pads occupied, checking if one of them is
			else:
				rospy.loginfo("Checking if user sits..")
				# Large sofa occupied?
				if(large_sofa_value != self.large_sofa_value):
					rospy.loginfo("Going to large sofa..")
					self.user_sat_down = True 
					self.large_sofa_value = large_sofa_value
					self.ctrs.move("base","large_sofa_offer", False)
				# Small sofa occupied?
				elif(small_sofa_value != self.small_sofa_value):
					rospy.loginfo("Going to small sofa..")
					self.user_sat_down = True
					self.small_sofa_value = small_sofa_value
					self.ctrs.move("base","small_sofa_offer", False)

		# User sat down, robot at user location, user picked drink out of the holder
		# Time to return to the charging station
		elif(tray_value != self.tray_value and self.user_sat_down == True):
			rospy.loginfo("Delivered drink to the user, now heading to charging station..")
			self.ctrs.move("base","charging_station", False)
			self.fridge_value = fridge_value
			self.user_sat_down = False
			self.fridge_opened = False
			self.drink_placed = False
			self.tray_value = tray_value

if __name__ == '__main__':
	rospy.loginfo("Starting demo")
	demo2 = Demo2()
	while not rospy.is_shutdown():
		demo2.run()
		time.sleep(1)
	rospy.loginfo("Demo stopped..")
