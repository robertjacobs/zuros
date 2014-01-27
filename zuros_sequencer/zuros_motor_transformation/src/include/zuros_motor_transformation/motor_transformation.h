#ifndef __DPR2_BASE_H
#define __DPR2_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/// Basic DPR2 base controller class
/**
 * This class reads the following values from the parameter server:
 * \param ~/motor_port The topic name of the \c shared_serial node used for communication with the motor
 * \param ~/motor_config The name of the motor configuration XML file
 * \param ~/wheel_diameter The diameter of the wheels in [m]
 * \param ~/wheel_base The distance between the wheels in [m]
 */
class ZurosBase
{
protected:
	ros::NodeHandle nh_;
	ros::Subscriber vel_sub_;
	ros::Publisher odom_pub_;

	LxSerial _serial_port;
	CDxlGeneric *_motor_left, *_motor_right;
	double wheel_diameter_ ,wheel_base_ ;

	ros::Time current_time_, last_time_;

	tf::TransformBroadcaster odom_broadcaster_;
	ros::ServiceServer set_angle_server_, set_distance_server_;

	double x_;
	double y_;
	double dist_;
	double th_;
	double vx_;
	double vth_;
	
protected:
	/// Called when a new velocity command is published
	/**
	 * Sends the new velocity to the base motors
	 * \param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
	 * \note Since the base is nonholonomic, only linear velocities in the x direction and angular velocities around the z direction are supported.
	 */
	void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg);

	/**
	 * Sends the status of the base motors
	 */
	void statusPublish();
	void odometryPublish();

public:
	DPR2Base(ros::NodeHandle nh)
	{
		nh_ = nh;
		init();
	}

	~DPR2Base()
	{
		delete _motor_left;
		delete _motor_right;

		nh_.shutdown();
	}

	/// Initialize the base motors
	/** \note Called during construction */
	void init();

	/// Await and process commands
	void spin();
};

#endif /* __DPR2_BASE_H */

