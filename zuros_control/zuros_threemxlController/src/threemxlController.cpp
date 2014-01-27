#include "zuros_threemxl_controller/threemxlController.h"
#include <threemxl/C3mxlROS.h>
#include <threemxl/LxFTDI.h>
#include <threemxl/dxlassert.h>
#define CLIP(x, l, u) ((x)<(l))?(l):((x)>(u))?(u):(x)

void DPR2Base::init()
{
	ROS_INFO("Initializing base");

	// Subscript to command topic
	vel_sub_ = nh_.subscribe("movement", 10, &DPR2Base::velocityCallback, this);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

	wheel_diameter_ = 0.297;
	wheel_base_ = 0.40;

	ros::Rate init_rate(1);

	CDxlConfig *config_left_motor = new CDxlConfig();
	_serial_port.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
     _serial_port.set_speed(LxSerial::S921600);

	if(_serial_port.is_port_open())
	{
		ROS_INFO("SERIAL PORT IS OPEN");
	}

	// Initialize left motor
	left_motor_ = new C3mxl();
	left_motor_->setSerialPort(&_serial_port);
	config_left_motor->setID(106);
	left_motor_->setConfig(config_left_motor);

	DXL_SAFE_CALL(left_motor_->set3MxlMode(SPEED_MODE));

	ROS_INFO("Left motor mode set");

	while (ros::ok() && left_motor_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize left wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	// Initialize right motor
	CDxlConfig *config_right_motor = new CDxlConfig();
	right_motor_ = new C3mxl();
	right_motor_->setSerialPort(&_serial_port);
	config_right_motor->setID(107);
	right_motor_->setConfig(config_right_motor);

	while (ros::ok() && right_motor_->set3MxlMode(SPEED_MODE) != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize right wheel motor mode, will continue trying every second");
		init_rate.sleep();
	}

	ROS_INFO("Right motor mode set");

	while (ros::ok() && right_motor_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
		init_rate.sleep();
	}
}

void DPR2Base::spin()
{
	ROS_INFO("Spinning");
	ros::Rate r(100);

	while(ros::ok())
	{
		ros::spinOnce();
		odometryPublish();
		r.sleep();
	}

	_serial_port.port_close();
}

void DPR2Base::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	// Base is nonholonomic, warn if sent a command we can't execute
	if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y)
	{
		ROS_WARN("It is not possible for me to translate this command to the robot - maybe you set the base type in navigation config to holonomic instead of nonholonomic or vice versa?");
		return;
	}

	if(isnan(msg->linear.x) || isnan(msg->angular.z) || isinf(msg->linear.x) || isinf(msg->angular.z))
	{
		ROS_WARN("I cant travel at infinite speed. Sorry");
		return;
	}

	// Calculate wheel velocities
	double vel_linear  = msg->linear.x/(wheel_diameter_/2);
	double vel_angular = msg->angular.z * (wheel_base_/wheel_diameter_);

	double vel_left    = vel_linear - vel_angular;
	double vel_right   = vel_linear + vel_angular;

	// Actuate
	left_motor_->setSpeed(vel_left);
	right_motor_->setSpeed(vel_right);

	ROS_DEBUG_STREAM("Base velocity set to [" << vel_left << ", " << vel_right << "]");
}

// reads the current change in wheel and publish as odometry
void DPR2Base::odometryPublish()
{
	left_motor_->getState();
	right_motor_->getState();

	double left = left_motor_->presentSpeed();
	double right = right_motor_->presentSpeed();

	vx_ = (wheel_diameter_ / 4) * (left + right);
	vth_ = ((wheel_diameter_ / 2) / wheel_base_) * (right - left);

	current_time_ = ros::Time::now();

	//compute odometry in a typical way given the velocities of the robot
	double dt = (current_time_ - last_time_).toSec();
	double delta_x = (vx_ * cos(th_)) * dt;
	double delta_y = (vx_ * sin(th_)) * dt;
	double delta_th = vth_ * dt;
	double delta_dist_ = vx_ *dt;

	x_ += delta_x;
	y_ += delta_y;
	th_ += delta_th;
	dist_ += delta_dist_;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time_;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x_;
	odom_trans.transform.translation.y = y_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster_.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time_;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	//set the position
	odom.pose.pose.position.x = x_;
	odom.pose.pose.position.y = y_;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.twist.twist.linear.x = vx_;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth_;

	//publish the message
	odom_pub_.publish(odom);

	last_time_ = current_time_;
}
