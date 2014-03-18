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
Will read and publish laptop battery information
Battery information is being polled by the command gui, but you can also use rostopic echo laptop/status/battery
"""

import roslib; roslib.load_manifest('zuros_laptop')
import rospy
from zuros_laptop.msg import MSG_LAPTOP_BATTERY
import os
import re

## Will get the laptop information and publish it on the topic
def get_and_publish():
    # Init rospy publisher and node
    pub = rospy.Publisher('laptop/status/battery', MSG_LAPTOP_BATTERY)
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
        str_remaining = remaining2[index+1:len(remaining2)][1:][:-1]
        
        # Publish the battery info on the topic
        pub.publish(MSG_LAPTOP_BATTERY(name=str_battery_name, state=str_state, percentage=string_percentage, remaining=str_remaining))
        rospy.sleep(0.5)

## Check if this is a class call or a program call
if __name__ == '__main__':
    try:
        get_and_publish()
    except rospy.ROSInterruptException:
        print "Program stopped manually.."
        pass
