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
This module handles the polling of sensors. It is built in a way that multiple sensor networks can be used, but this time the only network used is zwave.
"""

import sys
from socket import AF_INET, SOCK_DGRAM, socket, timeout
import json, urllib2, base64

import rospy
from zuros_environment_sensors.msg import MSG_ZWAVE_SENSORS, MSG_ZWAVE_STATUS

#polling processor made by UH and previously used with care-o-bot
#https://github.com/uh-adapsys/UHCore/blob/master/Core/extensions.py
from include.extensions import PollingProcessor

## Polls the fibaro zwave api and constantly checks for new sensors and changed sensor values.
#
# If a new sensor is added, the class broadcasts on ROS topic ZWAVE_SENSORS
# If a new value is detected, the class broadcasts on ROS topic ZWAVE_STATUS
#
# @param PollingProcessor The polling processor
class FibaroZWaveHomeController(PollingProcessor):
    # Initialization method
    def __init__ (self, ipAddress):
        super(FibaroZWaveHomeController, self).__init__()
        
        # Url of the ZWAVE controller
        self._baseUrl = ipAddress
        
        # Warnings regarding sensors gets placed inside this array so that terminal does not get flooded
        self._warned = []

        # Holds the added sensors
        self._sensors_added = []

        # The topic publishers
        self._sensor_publisher = rospy.Publisher('ZWAVE_SENSORS', MSG_ZWAVE_SENSORS)
        self._status_publisher = rospy.Publisher('ZWAVE_STATUS', MSG_ZWAVE_STATUS)

    ## Start function
    #
    # This function initializes the rospy node and starts the polling processor at a rate of 10Hz 
    def start(self):
        rospy.init_node('zuros_evironment_sensors_environment_sensors_py')
        rospy.loginfo(rospy.get_name() + " Started polling zwave sensors")
                
        self._addPollingProcessor('zwave', self.poll_zwave_sensors, None, 0.1)

    ## Stop function
    #
    # This function stops the polling processor
    def stop(self):
        rospy.loginfo(rospy.get_name() + " Stopped polling zwave sensors")
        self._removePollingProcessor('zwave')
    
    ## Polling function
    #
    # Polls the zwave api and handles data
    def poll_zwave_sensors(self):

        # Connect to the fibaro json api and pull the data
        try:
            # http://192.168.1.109/api/devices
            url = self._baseUrl
            request = urllib2.Request(url)
            base64string = base64.encodestring('%s:%s' % ('admin', 'admin')).replace('\n', '')
            request.add_header("Authorization", "Basic %s" % base64string)
            result = urllib2.urlopen(request)
            data = json.load(result)
        except Exception as e:
            if id(self) + type(e) not in self._warned:
                rospy.logerror(rospy.get_name() + " Error while receiving data from ZWaveHomeController: %s" % e)
                self._warned.append(id(self) + type(e))
                return
        
        #Analyzes the devices in the zwave controller. If there is a new, unknown, sensor, the StopIteration exception is raised
        #    the StopIteration exception is then used to add the new sensor to the list
        for device in data:
            # Format the communication ID to ZWAVE:[id]
            communication_id = "ZWAVE:" + str(device['id'])
            
            try:
                # The fibaro home controller has 3 "devices", which are no real devices, called admin, weather and zwave. Need to filter those since they are of no interest to us
                if(str(device['name']) != str("admin") and str(device['name']) != str("weather") and str(device['name']) != str("zwave")):
                    sensor = next(s for s in self._sensors_added if str(s['id']) == str(communication_id))

            except StopIteration:
                device['id'] = communication_id
                self._sensors_added.append(device)
                rospy.loginfo(rospy.get_name() + " Added sensor with name %s" % device['name'])
                continue

        #Checks the devices for changed values
        #    If a value was changed, publish to topic
        for device in data:
            # Format the communication ID to ZWAVE:[id]
            communication_id = "ZWAVE:" + str(device['id'])
            for sensor in self._sensors_added:
                if(str(sensor['id']) == str(communication_id)):
                    if(str(sensor['properties']['value']) != str(device['properties']['value'])):
                        sensor['properties']['value'] = device['properties']['value']
                        self._status_publisher.publish(MSG_ZWAVE_SENSORS(name=sensor['name'],value=sensor['properties']['value'],communication_id=sensor['id']))
                        rospy.loginfo(rospy.get_name() + " Value changed for device %s" % device['name'])
        
        # Publishes every sensor to topic so that other node knows which sensors are available
        for sensor in self._sensors_added:
            self._sensor_publisher.publish(MSG_ZWAVE_STATUS(name=sensor['name'],value=sensor['properties']['value'],communication_id=sensor['id']))

## Check if this is a class call or a program call
if __name__ == '__main__':
    import include.config
    # Array which holds the different sensors
    sensorList = []
    
    # Find sensors in location named ZAP in config file
    for key, sensorType in include.config.sensor_config['locations']['ZAP'].items():
        sensor = None

        # Is there a sensor with the name "ZWaveHomeController"?
        if sensorType == 'ZWaveHomeController':
            sensor = FibaroZWaveHomeController(include.config.sensor_config['sensor_config']['zwave']['server_address'])

        # Add your sensor type here

        # If we found a sensor, add it to the list
        if sensor != None:
            sensorList.append(sensor)

        else:
            rospy.loginfo(rospy.get_name() + " You have a sensor type in your config which you either forgot to add to the code or you made a typo")
    
    # For each sensor in the list, start the polling
    for sensor in sensorList:
        sensor.start()
    
    rospy.spin()
    
    # If there was a keyboard interrupt, stop all polling before shutdown
    for sensor in sensorList:
        sensor.stop()
