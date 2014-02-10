/*
* movement.h
*
* Created on: Jan 13, 2014
* Author: Robert Jacobs
*/

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <zuros_threemxlController/motorMSG.h>
#include <ros/callback_queue.h>

class Movement
{
public:
	Movement(ros::NodeHandle nh);
	void init();
	void spin();
	void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
	void callback_joy(const sensor_msgs::Joy::ConstPtr& msg);
private:
	bool joystick_override_;
	bool joystick_override_was_active_;
protected:
    ros::NodeHandle nh_;
	ros::Publisher publisher_cmd_vel_mov_;
	ros::Subscriber subscriber_cmd_vel_;
	ros::Subscriber subscriber_joy_;
	geometry_msgs::Twist message_;
};

#endif /* MOVEMENT_H_ */
