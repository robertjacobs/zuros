/********************************************************************* 
*
*  Copyright (c) 2013-2014 ZUYD Research
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*  Author Robert Jacobs/info@rjpjacobs.nl
*********************************************************************/

#include "zuros_threemxl_controller/threemxl_controller.h"
#include <threemxl/platform/io/configuration/XMLConfiguration.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/LxFTDI.h>
#include <threemxl/dxlassert.h>
#define CLIP(x, l, u) ((x)<(l))?(l):((x)>(u))?(u):(x)

/** Constructor
*/ 
void DPR2Base::init(std::string config_file_location)
{
  ROS_INFO("Initializing base");

  wheel_diameter_ = 0.30;
  wheel_base_ = 0.54;

  emergency_ = false;
  // Subscribe to emergency topic
  emergency_sub_ = nh_.subscribe("/emergency_stop", 10, &DPR2Base::emergencyCallback, this);

  // Subscribe to movement topic
  vel_sub_ = nh_.subscribe("/movement", 10, &DPR2Base::velocityCallback, this);

  // For the ROS navigation it is important to publish the odometry in the odom topic
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

  // The rate of retries in case of failure
  ros::Rate init_rate(1);

  //LXFTDI
  serial_port_ = new LxFTDI();
  serial_port_->port_open("i:0x0403:0x6001", LxSerial::RS485_FTDI);
  serial_port_->set_speed_int(921600);

  // Load motor configuration
  CXMLConfiguration motor_config_xml;
  ROS_ASSERT(motor_config_xml.loadFile(config_file_location));

  CDxlConfig motor_config_left;
  motor_config_left.readConfig(motor_config_xml.root().section("left"));

  // Left motor
  left_motor_ = new C3mxl();
  left_motor_->setConfig(&motor_config_left);
  left_motor_->setSerialPort(serial_port_);

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
  right_motor_->setSerialPort(serial_port_);

  // Initialize the right motor
  while (ros::ok() && right_motor_->init() != DXL_SUCCESS)
  {
    ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
    init_rate.sleep();
  }

  ROS_INFO("Right motor initialized");

  ROS_INFO("Motors initialized, will start spinning");
}

/** Own implementation for ros::spin  
*/ 
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

/** Callback function for the emergency topic
*   @param msg The message on the topic
*/ 
void DPR2Base::emergencyCallback(const std_msgs::Bool::ConstPtr &msg)
{
  emergency_ = msg->data;
}

/** Called when a new velocity command is published and sends the new velocity to the base motors
* @param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
* @note Since the base is nonholonomic, only linear velocities in the x direction and angular velocities around the z direction are supported.
*/
void DPR2Base::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  try
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
    double vel_linear = msg->linear.x/(wheel_diameter_/2);
    double vel_angular = msg->angular.z * (wheel_base_/wheel_diameter_);

    double vel_left = vel_linear - vel_angular;
    double vel_right = vel_linear + vel_angular;

    // Actuate
    if(!emergency_)
    {
      left_motor_->setSpeed(vel_left);
      right_motor_->setSpeed(vel_right);
      ROS_DEBUG_STREAM("Base velocity set to [" << vel_left << ", " << vel_right << "]");
    }

    // Emergency stop is issued, but we are still allowed to drive backwards with the laser on the front of the robot
    else
    {
      // Want to drive backwards (maybe joystick override to get stuck robot unstuck?) ?
      if((vel_left < 0 && vel_right < 0) || (vel_left == 0 && vel_right == 0))
      {
        left_motor_->setSpeed(vel_left);
        right_motor_->setSpeed(vel_right);
      }

      // Does not want to drive backwards, command ignored
      else
      {
        left_motor_->setSpeed(0);
        right_motor_->setSpeed(0);
        ROS_INFO("Emergency stop issued, motor command ignored");
      }
    }
  }
  
  catch (int e)
    {
        ROS_INFO("An exception occurred in velocity callback");
    }
}

/** Reads the current state of the wheels and publishes this to the topic */
void DPR2Base::odometryPublish()
{
  try
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

  catch (int e)
    {
      ROS_INFO("An exception occurred in odometry publish");
    }
}
