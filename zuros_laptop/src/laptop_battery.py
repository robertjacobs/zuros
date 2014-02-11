#!/usr/bin/env python
import roslib; roslib.load_manifest('zuros_laptop')
import rospy
from std_msgs.msg import String
import os
import subprocess

def talker():
	pub = rospy.Publisher('laptop/status/battery_voltage', String)
	rospy.init_node('zuros_laptop_laptop_voltage_py')
	while not rospy.is_shutdown():
		value = os.popen('acpi').read()
		pub.publish(value)
		rospy.sleep(0.5)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		print "Program stopped manually.."
		pass
