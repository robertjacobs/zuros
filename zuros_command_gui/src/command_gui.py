#!/usr/bin/python
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
This will generate the command GUI on the screen.
For this to work you need to use the command_gui.launch file, since it uploads parameters to the parameter server.
"""
import roslib
roslib.load_manifest('zuros_command_gui')
from zuros_laptop.msg import MSG_LAPTOP_BATTERY
from std_msgs.msg import Bool
from buttons import *
import thread
import pygtk
pygtk.require('2.0')
import gtk
import os
import pynotify
import sys 
import signal
import rospy

## GTK panel class
# @param gtk.Frame The frame
class GtkPanel(gtk.Frame):
    def __init__(self, master=None, labeltext=""):
        gtk.Frame.__init__(self)
        self.set_label(labeltext)
        self.set_shadow_type(gtk.SHADOW_IN)
        self.vbox = gtk.VBox(False, 0)
        self.add(self.vbox)

    ## Add a button to the panel
    def add_button(self, text, command):
        but = gtk.Button(text)  
        # Connect the clicked event of the button
        but.connect("clicked", start_GTK, command)
        self.vbox.pack_start(but, False, False, 5)

## GTK starter
# @param widget The widget
# @param data The data
def start_GTK(widget, data):
    data()

## Command gui class
# This is the main class for the command gui. It also handles all the messaging and displaying of events.
class CommandGUI():
    ## Delete event handler
    # @param self The object pointer
    # @param widget The widget
    # @param event The event
    # @param data The data
    def delete_event(self, widget, event, data=None):
        # Close main window
        gtk.main_quit()
        return False

    ## Constructor
    def __init__(self):
        # Init ros node
        rospy.init_node('zuros_command_gui')
        # Create a subscriber for the emergency stop topic
        rospy.Subscriber("/emergency_stop", Bool, self.callback_emergency_stop)
        # Create a subscriber for the laptop battery status topic
        rospy.Subscriber("/laptop/status/battery", MSG_LAPTOP_BATTERY, self.callback_laptop_battery)
        
        # Initialise the pynotify handler
        pynotify.init ("zuros_command_gui")

        # Set the initial states
        self.battery_state = "Discharging"
        self.battery_low_notified = False
        self.battery_critical_notified = False
        self.emergency = False

        # Create the gtk window
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("delete_event", self.delete_event)
        self.window.set_title("zuros command gui")  
        self.window.set_size_request(1000, 500)
        vbox = gtk.VBox(False, 1)
        self.hbox = gtk.HBox(True, 10)
        vbox.pack_start(self.hbox, True, True, 0)

        # Use the button class in the buttons.py file
        b = Buttons()
        panels = b.panels

        # If the number of panels is greater or equal to 5, we just create X panels
        if(len(panels) >= 5):
            for pname, actions in panels:
                panel = GtkPanel(self, pname)
                for aname, func, args in actions:
                    # This is a lambda function which will call the method referenced by the button action
                    panel.add_button(text=aname, command=lambda f=func, a=args: f(*a))
                    self.hbox.pack_start(panel,True, True, 3)

        # If the number of panels is below 5, we add some dummy panels to prettyfy the gui
        else:
            for pname, actions in panels:
                panel = GtkPanel(self, pname)
                for aname, func, args in actions:
                    # This is a lambda function which will call the method referenced by the button action
                    panel.add_button(text=aname, command=lambda f=func, a=args: f(*a))
                    self.hbox.pack_start(panel,True, True, 3)

            # Create the dummys
            for columns in range (0, 5-len(panels)):
                panel = GtkPanel(self, "")
                self.hbox.pack_start(panel,True, True, 3)

        # Create the status bar
        self.status_bar = gtk.Statusbar()  
        context_id = self.status_bar.get_context_id("Statusbar")
        string = "Connected to $ROS_MASTER_URI=" + os.environ.get("ROS_MASTER_URI") + "              Waiting for battery info..."    
        self.status_bar.push(context_id, string)
        vbox.pack_start(self.status_bar, False, False, 0)     

        self.window.add(vbox)    
        self.window.show_all()

        # Create a non-blocking GUI (multi threaded)
        gtk.gdk.threads_init()
        gtk.gdk.threads_enter()
        gtk.main()
        gtk.gdk.threads_leave()  
  
    ## Handler for the laptop battery status topic
    # @param msg The message on the topic
    def callback_laptop_battery(self, msg):
        # Write the message contents to the status bar
        self.write_battery_info_to_status_bar(msg.name, msg.state, msg.percentage, msg.remaining)

    ## Handler for the emergency stop status topic
    # Will show a popup balloon on the pc screen if there is a status change
    # @param msg The message
    def callback_emergency_stop(self, msg):
        # There was no emergency stop situation, but now it is, notify and set emergency stop to true.
        if(msg.data == True and self.emergency == False):
            print "Emergency stop issued"
            self.emergency = True

            # Create a popup balloon on the screen
            n = pynotify.Notification("Emergency stop issued", "", "dialog-warning")
            n.set_timeout(1)
            n.show()
        # There was an emergency stop situation, but now it is over, notify and set emergency stop to false.
        elif(msg.data == False and self.emergency == True):
            print "Emergency stop released"
            self.emergency = False
            # Create a popup balloon on the screen
            n = pynotify.Notification("Emergency stop released", "", "dialog-ok")
            n.set_timeout(1)
            n.show()

    ## Will write the battery status to the status bar
    # @param name The name of the battery
    # @param state The state of the battery
    # @param percentage The percentage of the battery
    # @param remaining The remaining time of the battery
    def write_battery_info_to_status_bar(self, name, state, percentage, remaining):
        # Check if we are in emergency mode
        if(self.emergency == False):
            # Not in emergency mode, so we can write the battery status to the status bar
            gtk.threads_enter()
            context_id = self.status_bar.get_context_id("Statusbar")
            string = "Connected to $ROS_MASTER_URI=" + os.environ.get("ROS_MASTER_URI") + "              Status of " + name + " is: " + state + ", " + percentage + "%, " + remaining  
            self.status_bar.push(context_id, string)
            gtk.threads_leave()
        
        # Detect a status change of the battery (e.g. battery cable unplugged or plugged)
        if(state != self.battery_state):
            # The charging cable was disconnected, but is now connected
            if(state == "Charging"):
                # Create a popup balloon on the screen
                self.battery_critical_notified = False
                self.battery_low_notified = False
                n = pynotify.Notification("Laptop charging cable connected", "", "dialog-ok")
                n.set_timeout(1)
                n.show()

            # The charging cable was connected, but is now disconnected
            elif(state == "Discharging"):
                # Create a popup balloon on the screen
                n = pynotify.Notification("Laptop charging cable unplugged", "", "dialog-warning")
                n.set_timeout(1)
                n.show()

            # Update the battery state to the current state
            self.battery_state = state

        # If the battery is getting low, it is good to notify the operator
        if(percentage <= 15 and self.battery_low_notified == False):
            # Create a popup balloon on the screen
            n = pynotify.Notification("Laptop battery is running low", "", "dialog-warning")
            n.set_timeout(1)
            n.show()
            # Prevent popup spam
            self.battery_low_notified = True

        # If the battery is critically low, it is good to notify the operator
        elif(percentage <= 8 and self.battery_critical_notified == False):
            # Create a popup balloon on the screen
            n = pynotify.Notification("Laptop battery is running low", "", "dialog-warning")
            n.set_timeout(1)
            n.show()
            # Prevent popup spam
            self.battery_critical_notified = True

## The signal handler for the shutting down event
def signal_handler(signal, frame):
    print 'Program stopped manually...'
    # Stop the main window
    gtk.main_quit()

## Check if this is a class call or a program call
if __name__ == "__main__":
    #signal handler for when shutting down the GUI
    signal.signal(signal.SIGINT, signal_handler)

    #start the gui
    program = CommandGUI()

    #spin
    rospy.spin()
