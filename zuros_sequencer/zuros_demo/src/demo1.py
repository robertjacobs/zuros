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
This is demo will show the robot performing a bouncer like behaviour.
The robot will start driving forward until it sees an obstacle
If it sees an obstacle, the robot will stop and reverse
While reversing the robot will keep looking if the object is gone
If the object is gone, the robot will turn
The robot will start moving foward again
"""

import roslib
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time
import random

## Demo1 class
class Demo1(object):
    ## init function
    def __init__(self):
        # on shutdown, clean up
        rospy.on_shutdown(self.cleanup)
        rospy.init_node('demo1_node', anonymous=False)

        # We want to send commands to the motors, regarding the software architecture this should be done by publishing on the cmd_vel topic
        # Important note: you should NEVER publish on the cmd_vel topic when the ROS navigation is initialized. Since, for this demo, there is no navigation needed, we can safely publish on cmd_vel
        self.motor_pub = rospy.Publisher('cmd_vel', Twist)

        # subscribe to emergency stop topic - this will be used to check if there was an object (since emergency stop publishes "true" when there is something too close to the robot
        rospy.Subscriber("/emergency_stop", Bool, self.callback_emergency)
        self.emergency = False

    ## The cleanup function which will make sure that a twist message is published on script shutdown (so that the motors stop moving)
    def cleanup(self):
        # stop the robot
        twist = Twist()
        self.motor_pub.publish(twist)

    ## Emergency stop callback
    def callback_emergency(self, emergency):
        self.emergency = emergency.data

    ## Drive forward function
    # Will command the motors to drive forward
    def forward(self, speed):
        # create a twist message, fill in the details
        twist = Twist()
        twist.linear.x = speed;                     # our forward speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = 0;                        # no rotation
        self.motor_pub.publish(twist)

    ## Drive backward function
    # Will command the motors to drive backwards
    def backward(self, speed):
        # create a twist message, fill in the details
        twist = Twist()
        twist.linear.x = speed * -1;                # our backward speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = 0;                        # no rotation
        self.motor_pub.publish(twist)

    ## Turn left function
    # Will command the motors to turn left
    def turn_left(self,speed):
        # create a twist message, fill in the details
        twist = Twist()
        twist.linear.x = 0;                         # no forward or backward speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = speed;                    # rotation
        self.motor_pub.publish(twist)

    ## Turn right function
    # Will command the motors to turn right
    def turn_right(self,speed):
        # create a twist message, fill in the details
        twist = Twist()
        twist.linear.x = 0;                         # no forward or backward speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = speed * -1;               # rotation
        self.motor_pub.publish(twist)

    ## Stop function
    # Will command the motors to stop moving
    def stop(self):
        # create a twist message, fill in the details
        twist = Twist()
        twist.linear.x = 0;                           # no speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = 0;                        # no rotation
        self.motor_pub.publish(twist)

    ## The run function - defines the robot behavior
    def run(self, speed_x):
        while True:
            print "forward"

            turn_right = False            

            while (self.emergency == False):
                self.forward(speed_x)
                pass

            print "Found an object, stopping robot"

            # emergency issued, stop robot
            self.stop()

            print "Robot stopped"

            time.sleep(1)

            # robot has stopped, now back up until the object is gone
            while (self.emergency == True):
                self.backward(speed_x)

            print "Object is gone, stopping"

            self.stop()

            print "Robot stopped, turning"

            time.sleep(2)

            # object is gone, now turn

            # Create a (pseudo!) random number to decide if the robot should turn left or right
            rand = random.randint(0,9)

            # Turn right or left?
            if(rand >= 5):
                turn_right = True

            # Turn
            for x in range (0,3):
                # Left or right?
                if(turn_right):
                    self.turn_right(speed_x)
                else:
                    self.turn_left(speed_x)
                time.sleep(1)

            time.sleep(3)

            print "done turning"

            self.stop()

## Check if this is a class call or a program call
if __name__ == '__main__':
    demo1 = Demo1()
    demo1.run(0.4)
    rospy.spin()
