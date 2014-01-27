/*
* movement.cpp
*
* Created on: Jan 13, 2014
* Author: Robert Jacobs
*/

#include "zuros_movement/movement.h"
#include <sstream>

Movement::Movement(ros::NodeHandle nh)
{
	nh_ = nh;
}

void Movement::init()
{
	joystick_override_ = false;
	publisher_cmd_vel_mov_ = nh_.advertise<geometry_msgs::Twist>("/movement", 100);
	subscriber_cmd_vel_ = nh_.subscribe("/cmd_vel", 100, &Movement::callback_cmd_vel, this);
	subscriber_joy_ = nh_.subscribe("/joy", 100, &Movement::callback_joy, this);
	ROS_INFO("DONE_INIT");
}

void Movement::spin()
{	
	while (ros::ok())
	{
		if(joystick_override_)
		{
			publisher_cmd_vel_mov_.publish(message_);
		}
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}

	// After ending loop, send a motor stop command
	message_.linear.x = 0;
	message_.linear.y = 0;
	message_.linear.z = 0;

	message_.angular.x = 0;
	message_.angular.y = 0;
	message_.angular.z = 0;

	publisher_cmd_vel_mov_.publish(message_);
}

void Movement::callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
	if(!joystick_override_)
	{
		publisher_cmd_vel_mov_.publish(msg);
	}
}

void Movement::callback_joy(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[5] == 1 && !joystick_override_)
	{
		ROS_INFO("USER OVERRIDE ACTIVE");
		joystick_override_ = true;
	}

	else if(msg->buttons[5] == 0 && joystick_override_ == true)
	{
		ROS_INFO("USER OVERRIDE RELEASED");	
		joystick_override_ = false;
	}

	if(joystick_override_)
	{
		if(msg->axes[1] == 0 || msg->axes[1] == -0)
        {
            message_.linear.x = 0;
            message_.linear.x = (0);
        }
		
		else
		{
			message_.linear.x = msg->axes[1];
		}

		if(msg->axes[3] == 0 || msg->axes[3] == -0)
        {
            message_.angular.z = 0;
            message_.angular.z = (0);
        }
		
		else
		{
			message_.angular.z = msg->axes[3];
		}
	}
}
