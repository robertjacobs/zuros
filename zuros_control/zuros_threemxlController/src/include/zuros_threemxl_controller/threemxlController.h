#ifndef __DPR2_BASE_H
#define __DPR2_BASE_H

/** @brief Node for controlling and reading threemxl motor controller.

		This node takes care of the odometry value reading and publishing and also takes care of the translation of ROS navigation commands.
		The ROS navigation commands are normally published over the cmd_vel topic, but since this cmd_vel topic is handled in the movement node, this node subscribes to the movement topic
		@author Robert Jacobs
		@date Jan, 27 - 2014
*/

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <threemxl/LxFTDI.h>
#include "std_msgs/Bool.h"

/// Basic DPR2 base controller class
/**
 * This class reads the following values from the parameter server:
 * \param ~/motor_port The topic name of the \c shared_serial node used for communication with the motor
 * \param ~/motor_config The name of the motor configuration XML file
 * \param ~/wheel_diameter The diameter of the wheels in [m]
 * \param ~/wheel_base The distance between the wheels in [m]
 */
class DPR2Base
{
protected:
	ros::NodeHandle nh_; 							/** The ROS nodehandle */
	ros::Subscriber vel_sub_; 						/** Subscriber for velocity topic */
	ros::Publisher odom_pub_; 						/** Publisher for the odemetry status */

	ros::Subscriber emergency_sub_; 				/** Subscriber for the emergency stop topic */
	bool emergency_;

	LxSerial serial_port_; 							/** Serial port to communicate with the motors */
	CDxlGeneric *left_motor_, *right_motor_; 			/** Left and right motor objects */
	CDxlConfig *config_left_motor_, *config_right_motor_;  /** Left and right motor config objects */
	double wheel_diameter_, wheel_base_; 			     /** The platform wheel diameter and wheel base */

	ros::Time current_time_, last_time_; 				/** Used for publishing odometry */

	tf::TransformBroadcaster odom_broadcaster_; 			/** Broadcasts transform information */

	double x_;    									/** odometry X */
	double y_;    									/** odometry Y */
	double dist_; 									/** odemetry distance */
	double th_;   									/** odemetry distance */
	double vx_;   									/** velocity X */
	double vth_;  									/** velocity theta */
	
protected:
	/** Called when a new velocity command is published and sends the new velocity to the base motors
	 * @param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
	 * @note Since the base is nonholonomic, only linear velocities in the x direction and angular velocities around the z direction are supported.
	 */
	void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg);

	/** Called when a new emergency status message is published
	* @param msg Pointer to std_msgs:Bool message, containing the current status of the emergency stop.
	*/
	void emergencyCallback(const std_msgs::Bool::ConstPtr & msg);

	/** Reads the current state of the wheels and publishes this to the topic */
	void odometryPublish();

public:
	/** Default constuctor */
	DPR2Base(ros::NodeHandle nh)
	{
		nh_ = nh;
		init();
	}

	/** Destructor. Deletes the left and right motor object, closes the serial port and shuts the node down */
	~DPR2Base()
	{
		delete left_motor_;
		delete right_motor_;
		delete config_left_motor_;
		delete config_right_motor_;
	
		if(serial_port_.is_port_open())
		{
			serial_port_.port_close();
		}

		nh_.shutdown();
	}

	/** Init the motors */
	void init();

	/** Own implementation for ros::spin */
	void spin();
};

#endif /* __DPR2_BASE_H */

