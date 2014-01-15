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
	_node = nh;
}

void Movement::init()
{
	/**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. Messages are passed to a callback function, here
	* called receiveCallback. subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe. When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue. If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/
	_publisher = _node.advertise<zuros_threemxlController::motorMSG>("/zuros_motor", 100);
	_subscriber = _node.subscribe("/joy", 100, &Movement::receiveCallback, this);
	ROS_INFO("DONE_INIT");
}

void Movement::spin()
{	
	while (ros::ok())
	{
		if(_override)
		{
			_publisher.publish(_message);
		}
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}

	// After ending loop, send a motor stop command
	zuros_threemxlController::motorMSG temp_message;
	temp_message.left_accel = 0;
	temp_message.left_speed = 0;

	temp_message.right_accel = 0;
	temp_message.right_speed = 0;
	_publisher.publish(temp_message);
}

void Movement::receiveCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[5] == 1)
	{
		if(_override == false)
		{
			ROS_INFO("USER OVERRIDE ACTIVE");
			_override = true;
		}
	}

	else if(msg->buttons[5] == 0 && _override == true)
	{
		ROS_INFO("USER OVERRIDE RELEASED");
		
		//Stop motors		
		_message.left_accel = 0;
		_message.left_speed = 0;

		_message.right_accel = 0;
		_message.right_speed = 0;

		_override = false;
	}

	if(_override)
	{
		int speed = 3;

		if(msg->axes[5] < 0.8)
		{
			_message.left_accel = 6;

			_message.right_accel = 6;
			speed = 10;
		}

		else if(msg->axes[5] > 0.8)
		{	
			_message.left_accel = (3);

			_message.right_accel = (3);
		}

		if(msg->axes[1] == 0 || msg->axes[1] == -0)
		{
			_message.left_speed = 0;
			_message.left_accel = (0);
			
			_message.right_speed = 0;
			_message.right_accel = (0);
		}
		
		if(msg->axes[3] > 0 || msg->axes[3] < 0)
		{
			_message.left_speed = ((msg->axes[3] * speed) * -1);

			_message.right_speed = (msg->axes[3] * speed);
		}

		else
		{
			_message.left_speed = (msg->axes[1] * speed);

			_message.right_speed = (msg->axes[1] * speed);
		}
	}
}
