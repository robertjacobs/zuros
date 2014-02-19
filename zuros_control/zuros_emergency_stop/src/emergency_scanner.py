#!/usr/bin/env python
import roslib; roslib.load_manifest('zuros_emergency_stop')
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import os
import math

class EmergencyChecker(object):
	def __init__(self):
		self.nan_count = 0
		self.emergency_stop = False
		self.pub = rospy.Publisher('emergency_stop', Bool)

	def callback(self,data):
		self.nan_count = 0
		for r in range (0,640):
			if(math.isnan(data.ranges[r])):
				self.nan_count = self.nan_count + 1

		if(self.nan_count >= 110 and self.emergency_stop == False):
			self.emergency_stop = True
			rospy.loginfo("EMERGENCY STOP ISSUED")

		elif(self.nan_count <= 109 and self.emergency_stop == True):
			self.emergency_stop = False			
			rospy.loginfo("EMERGENCY STOP RELEASED")

		print self.nan_count

		self.pub.publish(Bool(self.emergency_stop))
			
if __name__ == '__main__':
	rospy.init_node('emergency_scanner', anonymous=True)
	emc = EmergencyChecker()	
	rospy.Subscriber("scan", LaserScan, emc.callback)
	rospy.spin()
