/*
* movement.h
*
* Created on: Jan 13, 2014
* Author: Robert Jacobs
*/

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <zuros_threemxlController/motorMSG.h>
#include <ros/callback_queue.h>

class Movement
{
public:
	Movement(ros::NodeHandle nh);
	void init();
	void spin();
	void receiveCallback(const sensor_msgs::Joy::ConstPtr& msg);
private:
	bool _override;
protected:
    ros::NodeHandle _node;
    ros::Subscriber _subscriber;
	ros::Publisher _publisher;
	zuros_threemxlController::motorMSG _message;
};

#endif /* MOVEMENT_H_ */
