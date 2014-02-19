#!/usr/bin/env python

import roslib; roslib.load_manifest('zuros_command_to_robot_sender')
import rospy
import actionlib

import thread

#move_base_msgs
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *

class CommandToRobotSender(object):
	def __init__(self):
		self.action_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		#self.pub_move_base_simple = rospy.Publisher("/move_base_simple/goal", PoseStamped)

	def SortDict(self,dictionary):
		keys = sorted(dictionary.iterkeys())
		k=[]
		return [[key,dictionary[key]] for key in keys]

	def move(self, component_name, parameter, blocking):
		if component_name == "base":
			if parameter == "stop":
				return self.stop(component_name)
			else:
				return self.move_base(component_name, parameter, blocking)
		else:
			rospy.logerror(rospy.get_name() + "The component requested is not yet implemented");

	def stop(self, component_name):
		#base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Stop <<%s>>", component_name)
		self.action_client.cancel_all_goals()
		
	def move_base(self, component_name, position, blocking):
		#ah = action_handle("move_base", component_name, position, blocking, self.parse)
		
		#look up position in parameter server
		nav_prefix = "~nav_positions"

		if not rospy.has_param(nav_prefix):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_prefix)
			return False

		navigation_positions_params = rospy.get_param(nav_prefix)
		nav_param = self.SortDict(navigation_positions_params)

		nav_pos = None		

		for nav in nav_param:
			if(nav[0] == position):			
				#print nav
				#print nav[0]
				#position = nav[1]
				#print position[0]
				#print position[1]
				#print position[2]
				#print "Param %s has positions: [x: %d, y: %d, z: %d]" % { nav[0], nav[1][0], nav[1][1], nav[1][2] }

				nav_pos = nav[1]
		
		if(nav_pos != None):
			rospy.loginfo("Move <<%s>> to <<[x,y,z] %d, %d, %d>>", component_name, nav_pos[0], nav_pos[1], nav_pos[2])
		else:
			ROS_ERROR("No valid position found, cancelling move command. Are you sure your position is added to the parameter server?")
			return

		# convert to pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "/map"
		pose.pose.position.x = nav_pos[0]
		pose.pose.position.y = nav_pos[1]
		pose.pose.position.z = 0
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = nav_pos[2]

		rospy.logdebug("waiting for move_base action server to start")
		if not self.action_client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("move_base action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("move_base action server ready")

		# sending goal
		client_goal = MoveBaseGoal()
		client_goal.target_pose = pose
		#print client_goal
		
		thread.start_new_thread( self.handle, (client_goal,))
		#if(self.action_client.get_result == 

		#self.pub_move_base_simple.publish(pose)

	def handle(self, goal):
		self.action_client.send_goal(goal)
		self.action_client.wait_for_result()
		print "result: "
		print self.action_client.get_result()
