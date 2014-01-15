/*
* 3mxlController.h
*
* Created on: Jan 13, 2014
* Author: Robert Jacobs
*/

#ifndef ZUROS_3MXL_CONTROLLER_H_
#define ZUROS_3MXL_CONTROLLER_H_

#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <threemxl/C3mxlROS.h>
#include <zuros_threemxlController/motorMSG.h>

class threemxlController
{
public:
	threemxlController(ros::NodeHandle nh);
	void init();
	void receiveCallback(const zuros_threemxlController::motorMSG::ConstPtr& msg);

protected:
	ros::NodeHandle _node;
	ros::Subscriber _subscriber;
	CDxlGeneric *_motor_left;
	CDxlGeneric *_motor_right;
	LxSerial _serial_port; ///< Serial port interface
};


#endif /* ZUROS_3MXL_CONTROLLER_H_ */
