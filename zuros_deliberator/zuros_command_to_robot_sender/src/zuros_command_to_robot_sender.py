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
This will make sure that the commands, as instructed by the user using the command gui, are sent to the robot
"""

import roslib; roslib.load_manifest('zuros_command_to_robot_sender')
import rospy
import tf
import math
import actionlib

import thread

#move_base_msgs
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *

## Command to robot sender class
class CommandToRobotSender(object):
    ## Constructor
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        #self.pub_move_base_simple = rospy.Publisher("/move_base_simple/goal", PoseStamped)

    ## Sorts a dictionary alphabetically
    def sort_dict(self,dictionary):
        keys = sorted(dictionary.iterkeys())
        k=[]
        return [[key,dictionary[key]] for key in keys]

    ## The move method. Currently only base implemented, for further hardware you must implement
    def move(self, component_name, parameter, blocking):
        # Is this a base command?
        if component_name == "base":
            # Stop command?
            if parameter == "stop":
                return self.base_stop(component_name)

            # Not a stop command, so it should be a move base command
            else:
                return self.move_base(component_name, parameter, blocking)
        
        # Add your own component here
        # if component_name == "my_component":

        # No valid component (not implemented? Typo?)
        else:
            rospy.logerror(rospy.get_name() + "The component requested is not yet implemented");

    ## Base stop function - gets called if the component name is "base" and the parameter is "stop" in the move function above
    def base_stop(self, component_name):
        #base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Stop <<%s>>", component_name)
        self.action_client.cancel_all_goals()
        
    ## Move base funtion
    def move_base(self, component_name, position, blocking):
        #ah = action_handle("move_base", component_name, position, blocking, self.parse)
        
        # Look up position in parameter server
        nav_prefix = "~nav_positions"

        # Not on parameter server?
        if not rospy.has_param(nav_prefix):
            rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_prefix)
            return False

        # Get parameters
        navigation_positions_params = rospy.get_param(nav_prefix)
        nav_param = self.sort_dict(navigation_positions_params)

        nav_pos = None

        # Check if this position is known
        for nav in nav_param:
            if(nav[0] == position):
                nav_pos = nav[1]
        
        # Position is known
        if(nav_pos != None):
            rospy.loginfo("Move <<%s>> to <<[x,y,yaw] %d, %d, %d>>", component_name, nav_pos[0], nav_pos[1], nav_pos[2])
        
        # Position is not known
        else:
            ROS_ERROR("No valid position found, cancelling move command. Are you sure your position is added to the parameter server?")
            return

        # Convert to pose message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/map"
        pose.pose.position.x = nav_pos[0]
        pose.pose.position.y = nav_pos[1]
        pose.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, nav_pos[2])

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        rospy.logdebug("waiting for move_base action server to start")
        # Error: server did not respond within given time
        if not self.action_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("move_base action server not ready within timeout, aborting...")
            return
        else:
            rospy.logdebug("move_base action server ready")

        # sending goal
        client_goal = MoveBaseGoal()
        client_goal.target_pose = pose
        
        thread.start_new_thread( self.handle, (client_goal,))

        #self.pub_move_base_simple.publish(pose)

    ## Handle function which sends the command to the action server
    def handle(self, goal):
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
