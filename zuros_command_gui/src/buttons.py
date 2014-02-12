import rospy
from zuros_command_to_robot_sender import *

class Buttons(object):
	def __init__(self):
		self.groups = []
		self.buttons = []
		self.panels = []
		self.cmtrs = CommandToRobotSender()

		param_prefix = "~nav_buttons"
		if not rospy.has_param(param_prefix):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_prefix)
			return False
		group_param = rospy.get_param(param_prefix)
		group_param = self.SortDict(group_param)

		for group in group_param:
			group_name = group[1]["group_name"]
			component_name = group[1]["component_name"]
			nav_button_list = group[1]["buttons"]		

			for button in nav_button_list:
				self.buttons.append(self.CreateButton(button[0],self.cmtrs.move,component_name,button[2],button[3]))
			group = (group_name,self.buttons)
			self.panels.append(group)

	def move(self,component_name,parameter_name,blocking=True):
		print "Clicked: %s" % parameter_name

	def CreateButton(self,button_name,function,component_name,parameter_name, blocking):
		button = (button_name,function,(component_name,parameter_name, blocking))
		return button

	## Sorts a dictionary alphabetically
	def SortDict(self,dictionary):
		keys = sorted(dictionary.iterkeys())
		k=[]
		return [[key,dictionary[key]] for key in keys]
