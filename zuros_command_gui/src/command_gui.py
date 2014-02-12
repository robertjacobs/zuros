#!/usr/bin/python
import roslib
roslib.load_manifest('zuros_command_gui')
from zuros_laptop.msg import MSG_LAPTOP_BATTERY
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

class GtkPanel(gtk.Frame):
	def __init__(self, master=None, labeltext=""):
		gtk.Frame.__init__(self)
		self.set_label(labeltext)
		self.set_shadow_type(gtk.SHADOW_IN)
		self.vbox = gtk.VBox(False, 0)
		self.add(self.vbox)

	def addButton(self, text, command):
		but = gtk.Button(text)  
		but.connect("clicked", startGTK, command)
		self.vbox.pack_start(but, False, False, 5)

def startGTK(widget, data):
	data()

## Implementation of command gui
class CommandGUI():
	def delete_event(self, widget, event, data=None):
		gtk.main_quit()
		return False
    
	def CallbackLaptopBattery(self, msg):
		self.WriteBatteryInfoToStatusBar(msg.name, msg.state, msg.percentage, msg.remaining)

	def WriteBatteryInfoToStatusBar(self, name, state, percentage, remaining):
		gtk.threads_enter()
		context_id = self.status_bar.get_context_id("Statusbar")
		string = "Connected to $ROS_MASTER_URI=" + os.environ.get("ROS_MASTER_URI") + "              Status of " + name + " is: " + state + ", " + percentage + "%, " + remaining  
		self.status_bar.push(context_id, string)
		gtk.threads_leave()
		
		if(state != self.battery_state):
			if(state == "Charging"):
				n = pynotify.Notification("Laptop charging cable connected", "", "dialog-ok")
				n.set_timeout(1)
				n.show()

		elif(state == "Discharging"):
			n = pynotify.Notification("Laptop charging cable unplugged", "", "dialog-warning")
			n.set_timeout(1)
			n.show()

		self.battery_state = state

		if(percentage <= "15" and self.battery_low_notified == False):
			n = pynotify.Notification("Laptop battery is running low", "", "dialog-warning")
			n.set_timeout(1)
			n.show()
			self.battery_low_notified = True

		elif(percentage <=8 and self.battery_critical_notified == False):
			n = pynotify.Notification("Laptop battery is running low", "", "dialog-warning")
			n.set_timeout(1)
			n.show()
			self.battery_critical_notified = True

	def __init__(self):
		#init ros node
		rospy.init_node('zuros_command_gui')
		rospy.Subscriber("/laptop/status/battery", MSG_LAPTOP_BATTERY, self.CallbackLaptopBattery)
		pynotify.init ("zuros_command_gui")
		self.battery_state = "Discharging"
		self.battery_low_notified = False
		self.battery_critical_notified = False

		self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
		self.window.connect("delete_event", self.delete_event)
		self.window.set_title("zuros command gui")  
		self.window.set_size_request(1000, 500)
		vbox = gtk.VBox(False, 1)
		self.hbox = gtk.HBox(True, 10)
		vbox.pack_start(self.hbox, True, True, 0)

		b = Buttons()
		panels = b.panels

		if(len(panels) >= 5):
			for pname, actions in panels:
				panel = GtkPanel(self, pname)
				for aname, func, args in actions:
					panel.addButton(text=aname, command=lambda f=func, a=args: f(*a))
					self.hbox.pack_start(panel,True, True, 3)

		else:
			for pname, actions in panels:
				panel = GtkPanel(self, pname)
				for aname, func, args in actions:
					panel.addButton(text=aname, command=lambda f=func, a=args: f(*a))
					self.hbox.pack_start(panel,True, True, 3)

			for columns in range (0, 5-len(panels)):
				panel = GtkPanel(self, "")
				self.hbox.pack_start(panel,True, True, 3)

		self.status_bar = gtk.Statusbar()  
		context_id = self.status_bar.get_context_id("Statusbar")
		string = "Connected to $ROS_MASTER_URI=" + os.environ.get("ROS_MASTER_URI") + "              Waiting for battery info..."    
		self.status_bar.push(context_id, string)
		vbox.pack_start(self.status_bar, False, False, 0)     
		self.window.add(vbox)    
		self.window.show_all()

		gtk.gdk.threads_init()
		gtk.gdk.threads_enter()
		gtk.main()
		gtk.gdk.threads_leave()

def signal_handler(signal, frame):
	print 'Program stopped manually...'
	gtk.main_quit()

if __name__ == "__main__":
	signal.signal(signal.SIGINT, signal_handler)
	program = CommandGUI()
