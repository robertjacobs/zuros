#!/usr/bin/env python

## @package zwavePoller.py
# This module handles writing the sensor information coming from zuros_sensors
#
# Robert Jacobs 2013

import sys
from socket import AF_INET, SOCK_DGRAM, socket, timeout
import json, urllib2, base64
import time

#Database connection
from include.database import SensorsInDatabase

#ROS stuff
import rospy
import roslib
roslib.load_manifest('zuros_zwave_poller')
from zuros_environment_sensors.msg import MSG_ZWAVE_SENSORS, MSG_ZWAVE_STATUS

## Handles the message gathering and writing to the 
#
#
class MessageHandler(object):
	## Init function that also gathers sensors and sensorTypes from database
	#	
	def __init__(self):		
		#get the sensor definitions from the database
		self._sensors = SensorsInDatabase().GetAllSensors()
		#get the sensorTypes definitions from the database
		self._sensorTypes = SensorsInDatabase().GetAllSensorTypes()

		self._sensorsInTopic = []
	
	## Callback method for the status topic (ZWAVE_STATUS)
	#
	def CallbackStatus(self, data):
		try:
			sensor = next(s for s in self._sensors if str(s['communication_id']) == str(data.communication_id))
		
			for type in self._sensorTypes:
				#We will need to check which sensorType our sensor has in order to interpret the value
				if(str(sensor['sensor_type']) == str(type['id'])):
					#Add the uninterpreted value into the value field
					sensor['value'] = data.value
					#Set the last_interpreted value to the current one
					sensor['last_interpreted_value'] = sensor['interpreted_value']
					#set the last updated datetime
					sensor['last_updated'] = time.strftime('%Y-%m-%d %H:%M:%S')
				        
					#check if we have an analog sensor or not
					if(type['on_value'] != None and type['off_value'] != None):
						if(str(data.value) == "0"):
							#interpreted value - set to value set in database
							sensor['interpreted_value'] = type['off_value']
				                                                                
						elif (str(data.value) == "1"):
							##interpreted value - set to value set in database
							sensor['interpreted_value'] = type['on_value']
					#we had an analog sensor, so we will write the raw value into the value
					else:
						sensor['interpreted_value'] = data.value
		    
			#Write the new sensor information to the database                                                
			SensorsInDatabase().UpdateSensorValue(sensor)
			rospy.loginfo(rospy.get_name() + ": Sensor status update (%s): %s" % (sensor['name'], sensor['interpreted_value']))
		except StopIteration:
			rospy.loginfo(rospy.get_name() + ": Sensor update for unknown sensor with name (%s) and id %s - please add it to the database" % (data.name, data.communication_id))

if __name__ == '__main__':
	import include.config_zwave_poller
	
	handler = MessageHandler()
	
	#ROS node
	rospy.init_node('zuros_zwave_poller_zwave_poller_py', anonymous=False)
	rospy.Subscriber(include.config_zwave_poller.zwave['message_status'], MSG_ZWAVE_STATUS, handler.CallbackStatus)
	
	#Not used at this point. In the future this could be used to dynamically check if a new sensor was added during operation
	#rospy.Subscriber(config_zwave_poller.zwave['message_sensors'], MSG_ZWAVE_SENSORS, handler.CallbackSensors)

	rospy.spin()
