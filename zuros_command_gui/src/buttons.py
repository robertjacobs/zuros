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
This will generate the buttons for the command GUI.
"""

import rospy
from zuros_command_to_robot_sender import *

## Button class
# This class makes the connection between the clicking of a button on the GUI and the ROS system
class Buttons(object):
    ## Constructor
    def __init__(self):
        self.groups = []
        self.buttons = []
        self.panels = []
        # This will handle the calls to ROS
        self.cmtrs = CommandToRobotSender()

        # Get parameters from the parameter server
        param_prefix = "~nav_buttons"
        if not rospy.has_param(param_prefix):
            rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_prefix)
            return False
        group_param = rospy.get_param(param_prefix)
        group_param = self.sort_dict(group_param)

        # Handle the parameters
        for group in group_param:
            group_name = group[1]["group_name"]
            component_name = group[1]["component_name"]
            nav_button_list = group[1]["buttons"]

            # Add buttons to column
            for button in nav_button_list:
                # This will create the button and attach the event to the button
                self.buttons.append(self.create_button(button[0],self.cmtrs.move,component_name,button[2],button[3]))
            group = (group_name,self.buttons)
            self.panels.append(group)

    ## Creates a button and attaches the clicked function reference
    def create_button(self,button_name,function,component_name,parameter_name, blocking):
        button = (button_name,function,(component_name,parameter_name, blocking))
        return button

    ## Sorts a dictionary alphabetically
    def sort_dict(self,dictionary):
        keys = sorted(dictionary.iterkeys())
        k=[]
        return [[key,dictionary[key]] for key in keys]
