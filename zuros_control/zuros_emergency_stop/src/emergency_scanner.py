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
This will check the scan topic and looks if there are a lot of NaN (Not a Number) values 
If there are too much NaN's, there is probably an object near -> emergency stop
"""

import roslib; roslib.load_manifest('zuros_emergency_stop')
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import os
import math

## Class for checking the status of the scan topic depth points
class EmergencyChecker(object):
    ## Constructor
    def __init__(self):
        self.nan_count = 0
        self.emergency_stop = False
        # Create a publisher for the emergency stop topic
        self.pub = rospy.Publisher('emergency_stop', Bool)

    ## Callback method
    # Gets called when there is new data in the topic
    def callback_scan(self,data):
        self.nan_count = 0
        # Check whole width of scan (scan is 640*480 points)
        for r in range (0,640):
            # If the point is not a number, this is potential risk
            if(math.isnan(data.ranges[r])):
                self.nan_count = self.nan_count + 1

        # If there are too many potential risks, trigger emergency stop
        if(self.nan_count >= 110 and self.emergency_stop == False):
            self.emergency_stop = True
            rospy.loginfo("EMERGENCY STOP ISSUED")

        # Untrigger emergency stop
        elif(self.nan_count <= 109 and self.emergency_stop == True):
            self.emergency_stop = False
            rospy.loginfo("EMERGENCY STOP RELEASED")

        # Publish status on topic
        self.pub.publish(Bool(self.emergency_stop))

## Check if this is a class call or a program call
if __name__ == '__main__':
    rospy.init_node('emergency_scanner', anonymous=True)
    # Start
    emc = EmergencyChecker()
    rospy.Subscriber("scan", LaserScan, emc.callback_scan)
    # Spin
    rospy.spin()
