#!/usr/bin/env python
import roslib
import rospy
import time
import MySQLdb

roslib.load_manifest('zuros_demo')

from include.database import SensorsInDatabase
from zuros_command_to_robot_sender import CommandToRobotSender

class Demo2(object):
	def __init__(self):
		rospy.init_node('demo2_node', anonymous=False)
		rospy.on_shutdown(self.cleanup)
		self.fridge_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:6")
		self.doorbell_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:26")
		self.fridge_opened = False
		self.ctrs = CommandToRobotSender()

	def cleanup(self):
		# stop the robot
		rospy.loginfo("Stopping now..")
		self.ctrs.move("base","stop", False)	

	def run(self):
		fridge_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:6")
		doorbell_value = SensorsInDatabase().get_value_by_communication_id("ZWAVE:26")

		# Fridge opened?
		if(fridge_value != self.fridge_value and self.fridge_opened == False):
			rospy.loginfo("Fridge opened, heading there now..")
			self.ctrs.move("base","kitchen_fridge", False)
			self.fridge_opened = True
			self.fridge_value = fridge_value

		# Fridge was opened and the user sits. Time to bring the goodies!
		if(doorbell_value != self.doorbell_value and self.fridge_opened == True):
			rospy.loginfo("user sat down, heading to user..")
			self.ctrs.move("base","small_sofa_offer", False)
			self.fridge_opened = False
			self.doorbell_value = doorbell_value

		else:
			rospy.loginfo("Nothing to do for me :(")
		
		time.sleep(1)

if __name__ == '__main__':
	rospy.loginfo("Starting demo")
	demo2 = Demo2()
	while not rospy.is_shutdown():
		demo2.run()
	rospy.loginfo("Demo stopped..")
