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
This module handles writing the sensor information coming from zuros_sensors
"""

import sys
from socket import AF_INET, SOCK_DGRAM, socket, timeout
import json, urllib2, base64
import time

# Database connection
from include.database import SensorsInDatabase

import rospy
import roslib
roslib.load_manifest('zuros_zwave_poller')
from zuros_environment_sensors.msg import MSG_ZWAVE_SENSORS, MSG_ZWAVE_STATUS

## Handles the message gathering and writing to the database
class MessageHandler(object):
    ## Init function that also gathers sensors and sensorTypes from database
    def __init__(self):
        # Get the sensor definitions from the database
        self._sensors = SensorsInDatabase().get_all_sensors()
        # Get the sensorTypes definitions from the database
        self._sensorTypes = SensorsInDatabase().get_all_sensor_types()

        self._sensorsInTopic = []
    
    ## Callback method for the status topic (ZWAVE_STATUS)
    # @param data The data in the topic
    def callback_status(self, data):
        try:
            # Get the sensor
            sensor = next(s for s in self._sensors if str(s['communication_id']) == str(data.communication_id))
        
            # Get the sensor type
            for type in self._sensorTypes:
                # We will need to check which sensorType our sensor has in order to interpret the value
                if(str(sensor['sensor_type']) == str(type['id'])):
                    # Add the uninterpreted value into the value field
                    sensor['value'] = data.value
                    # Set the last_interpreted value to the current one
                    sensor['last_interpreted_value'] = sensor['interpreted_value']
                    # Set the last updated datetime
                    sensor['last_updated'] = time.strftime('%Y-%m-%d %H:%M:%S')
                        
                    # Check if we have an analog sensor or not
                    if(type['on_value'] != None and type['off_value'] != None):
                        if(str(data.value) == "0"):
                            # Interpreted value - set to value set in database
                            sensor['interpreted_value'] = type['off_value']
                        
                        elif (str(data.value) == "1"):
                            # Interpreted value - set to value set in database
                            sensor['interpreted_value'] = type['on_value']
                    # Ee had an analog sensor, so we will write the raw value into the value
                    else:
                        sensor['interpreted_value'] = data.value
            
            # Write the new sensor information to the database
            SensorsInDatabase().update_sensor_value(sensor)
            rospy.loginfo(rospy.get_name() + ": Sensor status update (%s): %s" % (sensor['name'], sensor['interpreted_value']))
        except StopIteration:
            rospy.loginfo(rospy.get_name() + ": Sensor update for unknown sensor with name (%s) and id %s - please add it to the database" % (data.name, data.communication_id))

## Check if this is a class call or a program call
if __name__ == '__main__':
    import include.config_zwave_poller
    
    handler = MessageHandler()
    
    # ROS node
    rospy.init_node('zuros_zwave_poller_zwave_poller_py', anonymous=False)
    rospy.Subscriber(include.config_zwave_poller.zwave['message_status'], MSG_ZWAVE_STATUS, handler.callback_status)
    
    # Not used at this point. In the future this could be used to dynamically check if a new sensor was added during operation
    #rospy.Subscriber(config_zwave_poller.zwave['message_sensors'], MSG_ZWAVE_SENSORS, handler.CallbackSensors)

    rospy.spin()
