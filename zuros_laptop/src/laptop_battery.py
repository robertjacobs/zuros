#!/usr/bin/env python
import roslib; roslib.load_manifest('zuros_laptop')
import rospy
from zuros_laptop.msg import MSG_LAPTOP_BATTERY
import os
import re

def GetAndPublish():
	# Init rospy publisher and node
	pub = rospy.Publisher('laptop/status/battery_voltage', MSG_LAPTOP_BATTERY)
	rospy.init_node('zuros_laptop_laptop_voltage_py')

	while not rospy.is_shutdown():
		value = os.popen('acpi').read()

		# Split the string so that we get the battery X value
		index = value.find(':')
		str_battery_name = value[0:index]

		# Removed first word, keep looking for the state
		remaining = value[index+1:len(value)]
		index = remaining.find(',')
		str_state = remaining[0:index][1:]

		# Removed first two words, keep looking for the percentage
		remaining2 = remaining[index+1:len(remaining)]
		index = remaining2.find(',')
		string_percentage = remaining2[0:index-1][1:]	

		# The only thing remaining, after removing the first words, is either nothing (will publish percentage) or the remaining battery time
		str_remaining = remaining2[index+1:len(remaining2)][1:]	
		
		# Publish the battery info on the topic
		pub.publish(MSG_LAPTOP_BATTERY(battery_name=str_battery_name, state=str_state, percentage=string_percentage, remaining=str_remaining))
		rospy.sleep(0.5)

if __name__ == '__main__':
	try:
		GetAndPublish()
	except rospy.ROSInterruptException:
		print "Program stopped manually.."
		pass
