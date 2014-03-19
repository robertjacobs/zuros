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
This will check the scan topic and looks if there are any -inf (minus infinite) values 
If there are too much -infs there is probably an object nearby -> emergency stop
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
    # Gets called when there is new data in the scan topic      
    def callback_scan(self, data):
        self.inf_count = 0
        for r in range (0,640):   
            #the scan publisher (zuros_depth) publishes -inf values for each point < 0.8     
            if(math.isinf(data.ranges[r])):
                self.inf_count = self.inf_count + 1

        if(self.inf_count >= 2 and self.emergency_stop == False):
            self.emergency_stop = True            
            rospy.loginfo("EMERGENCY STOP ISSUED")
        else:
            if(self.inf_count < 2 and self.emergency_stop == True):
              self.emergency_stop = False
              rospy.loginfo("EMERGENCY STOP RELEASED")

        self.pub.publish(Bool(self.emergency_stop))

## Check if this is a class call or a program call
if __name__ == '__main__':
    rospy.init_node('emergency_scanner', anonymous=False)
    # Start
    emc = EmergencyChecker()
    rospy.Subscriber("scan", LaserScan, emc.callback_scan)
    # Spin
    rospy.spin()
