#!/usr/bin/env python

import roslib; roslib.load_manifest('zuros_command_to_robot_sender')
import rospy
import actionlib

#move_base_msgs
from move_base_msgs.msg import *
import rospy
import actionlib

class CommandToRobotSender(object):
	def __init(self):
		pass

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
		base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Stop <<%s>>", component_name)
		base_client.cancel_all_goals()
		
	def move_base(self, component_name, position, blocking):
		base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)		

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
			return

		# convert to pose message
		#pose = PoseStamped()
		#pose.header.stamp = rospy.Time.now()
		#pose.header.frame_id = "/map"
		#pose.pose.position.x = param[0]
		#pose.pose.position.y = param[1]
		#pose.pose.position.z = 0.0
		#q = quaternion_from_euler(0, 0, param[2])
		#pose.pose.orientation.x = q[0]
		#pose.pose.orientation.y = q[1]
		#pose.pose.orientation.z = q[2]
		#pose.pose.orientation.w = q[3]

		#client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)

		#rospy.logdebug("waiting for %s action server to start",action_server_name)
		#if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
		#	rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
		#	ah.set_failed(4)
		#	return ah
		#else:
		#	rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		#client_goal = MoveBaseGoal()
		#client_goal.target_pose = pose
		#print client_goal
		#client.send_goal(client_goal)
