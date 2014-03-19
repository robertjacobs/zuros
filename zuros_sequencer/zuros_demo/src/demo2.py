#!/usr/bin/env python
import roslib
import rospy
import time
import MySQLdb

roslib.load_manifest('zuros_demo')

from include.database import SensorsInDatabase
from zuros_command_to_robot_sender import CommandToRobotSender
from random import randrange

class Demo2(object):
	def __init__(self):
		rospy.init_node('demo2_node', anonymous=False)
		rospy.on_shutdown(self.cleanup)
		self.fridge_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:6")
		self.tray_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:26")
		self.large_sofa_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:25")
		self.small_sofa_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:21")

		self.fridge_opened = False
		self.user_sat_down = False
		self.drink_placed = False
		self.ctrs = CommandToRobotSender()

	def cleanup(self):
		# stop the robot
		rospy.loginfo("Stopping now..")
		self.ctrs.move("base","stop", False)	

	def run(self):
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

		elif(tray_value != self.tray_value and self.user_sat_down == True):
			rospy.loginfo("Delivered drink to the user, now heading to charging station..")
			self.ctrs.move("base","charging_station", False)
			self.fridge_value = fridge_value
			self.user_sat_down = False
			self.fridge_opened = False
			self.drink_placed = False
			self.tray_value = tray_value
		
		time.sleep(1)

if __name__ == '__main__':
	rospy.loginfo("Starting demo")
	demo2 = Demo2()
	while not rospy.is_shutdown():
		demo2.run()
	rospy.loginfo("Demo stopped..")
