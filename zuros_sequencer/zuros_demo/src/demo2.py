#!/usr/bin/env python
import roslib
import rospy
import time
import MySQLdb

class Demo2(object):
	def __init__(self):
		rospy.init_node('demo2_node', anonymous=False)
		rospy.on_shutdown(self.cleanup)
		self.db = MySQLdb.connect(host="localhost", # your host, usually localhost
						user="john", # your username
						passwd="megajonhy", # your password
						db="jonhydb") # name of the data base
		self.cur = self.db.cursor() 

	def cleanup(self):
		# stop the robot
		pass	

	def GetSensorStateFromDB(self):
		self.cur.execute("SELECT * FROM Sensor")
		for row in self.cur.fetchall() :
   			print row[0]

if __name__ == '__main__':
	demo2 = Demo2()
	rospy.spin
