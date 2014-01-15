/*
* example_motor_controller.cpp
*
* Created on: Jan 13, 2014
* Author: Robert Jacobs
*/

#include <zuros_threemxl/threemxlController.h>
#include <sstream>
#include <threemxl/C3mxlROS.h>

threemxlController::threemxlController(ros::NodeHandle nh)
{
    _node = nh;
}

void threemxlController::init()
{
	// Threemxl init
	CDxlConfig *config = new CDxlConfig();
	_motor_left = new C3mxl();
	_motor_right = new C3mxl();
	_serial_port.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
	_serial_port.set_speed(LxSerial::S921600);
	_motor_left->setSerialPort(&_serial_port);
	_motor_right->setSerialPort(&_serial_port);

	_motor_left->setConfig(config->setID(106));
	_motor_right->setConfig(config->setID(107));
	_motor_left->init(false);
	_motor_right->init(false);
	_motor_left->set3MxlMode(SPEED_MODE);
	_motor_right->set3MxlMode(SPEED_MODE);

	delete config;

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
    _subscriber = _node.subscribe("/zuros_motor", 10, &threemxlController::receiveCallback, this);
}

void threemxlController::receiveCallback(const zuros_threemxlController::motorMSG::ConstPtr& msg)
{
    _motor_left->setAcceleration(msg->left_accel);
	_motor_left->setSpeed(msg->left_speed);

	_motor_right->setAcceleration(msg->right_accel);
	_motor_right->setSpeed(msg->right_speed);
}
