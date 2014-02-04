#include "zuros_threemxl_controller/threemxlController.h"
#include <threemxl/platform/io/configuration/XMLConfiguration.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/LxFTDI.h>
#include <threemxl/dxlassert.h>
#define CLIP(x, l, u) ((x)<(l))?(l):((x)>(u))?(u):(x)

/** Init the motors */
void DPR2Base::init()
{
	ROS_INFO("Initializing base");

	wheel_diameter_ = 0.30;
	wheel_base_ = 0.54;

	// Subscribe to movement topic
	vel_sub_ = nh_.subscribe("/movement", 10, &DPR2Base::velocityCallback, this);

	// For the ROS navigation it is important to publish the odometry in the odom topic	
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

	// The rate of retries in case of failure
	ros::Rate init_rate(1);

	// Open serial port and set the speed
	serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
     serial_port_.set_speed(LxSerial::S921600);

	// Check if the serial port is open
	while(ros::ok() && serial_port_.is_port_open() == false)
	{
		ROS_WARN_ONCE("Serial port seems to be closed, will continue trying every second");
	}

	// Load motor configuration
	CXMLConfiguration motor_config_xml;
	//ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));
	ROS_ASSERT(motor_config_xml.loadFile("motors.xml"));
	
	CDxlConfig motor_config_left;
	motor_config_left.readConfig(motor_config_xml.root().section("left"));
	// Left motor
	left_motor_ = new C3mxl();
	left_motor_->setConfig(&motor_config_left);
	left_motor_->setSerialPort(&serial_port_);

	// Initialize the left motor
	while (ros::ok() && left_motor_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize left wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	ROS_INFO("Left motor initialized");

	// Right motor
	CDxlConfig motor_config_right;
	motor_config_right.readConfig(motor_config_xml.root().section("right"));

	right_motor_ = new C3mxl();
	
	right_motor_->setConfig(&motor_config_right);
	right_motor_->setSerialPort(&serial_port_);

	// Initialize the right motor
	while (ros::ok() && right_motor_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	ROS_INFO("Right motor initialized");

	ROS_INFO("Motors initialized, will start spinning");
}

/** Own implementation for ros::spin */
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
}	

/** Called when a new velocity command is published and sends the new velocity to the base motors
	 * @param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
	 * @note Since the base is nonholonomic, only linear velocities in the x direction and angular velocities around the z direction are supported.
	 */
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

/** Reads the current state of the wheels and publishes this to the topic */
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
