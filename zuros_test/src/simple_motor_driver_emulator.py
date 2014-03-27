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
This will emulate a motor driver.
As soon as this motor driver gets commands, it will output those to the terminal
This file was made to proof that the system is able to work with other platforms as well
	To do this, this motor driver emulator shows that it is simple to create a new controller and use it with the system
"""

import rospy
import roslib; roslib.load_manifest('zuros_test')
from zuros_motor_transformation.msg import differential

class MotorEmulator():
    def __init__(self):
        # Give the node a name
        rospy.init_node('motor_driver_emulator', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Prevents terminal spam
        self.stopped = False
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        rospy.loginfo("I would send a stop command to the robot")
        rospy.sleep(1)

    def callback_differential(self,msg):
        if(msg.left_motor_speed > 0 and msg.right_motor_speed > 0):
            self.stopped = False
            rospy.loginfo("Forward")
        elif(msg.left_motor_speed < 0 and msg.right_motor_speed < 0):
            self.stopped = False
            rospy.loginfo("Backward")
        elif(msg.left_motor_speed < 0 and msg.right_motor_speed > 0):
            self.stopped = False
            rospy.loginfo("Left")
        elif(msg.left_motor_speed > 0 and msg.right_motor_speed < 0):
            self.stopped = False
            rospy.loginfo("Right")
        else:
            if(self.stopped == False):
                rospy.loginfo("Motors stop")
                self.stopped = True
 
if __name__ == '__main__':
	motor_emulator = MotorEmulator()
	rospy.Subscriber("/motor_transformation_differential", differential, motor_emulator.callback_differential)
	rospy.spin()
