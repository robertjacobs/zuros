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
#include "zuros_motor_transformation/differential.h"

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
  ros::NodeHandle nh_;               /** The ROS nodehandle */
  ros::Subscriber vel_sub_;          /** Subscriber for velocity topic */
  ros::Publisher odom_pub_;          /** Publisher for the odemetry status */

  ros::Subscriber emergency_sub_;    /** Subscriber for the emergency stop topic */
  bool emergency_;

  //LxSerial serial_port_;           /** Serial port to communicate with the motors */
  LxSerial *serial_port_;  
  CDxlGeneric *left_motor_, *right_motor_;               /** Left and right motor objects */
  CDxlConfig *config_left_motor_, *config_right_motor_;  /** Left and right motor config objects */
  double wheel_diameter_, wheel_base_;                   /** The platform wheel diameter and wheel base */

  ros::Time current_time_, last_time_;          /** Used for publishing odometry */

  tf::TransformBroadcaster odom_broadcaster_;   /** Broadcasts transform information */

  double x_;                         /** odometry X */
  double y_;                         /** odometry Y */
  double dist_;                      /** odemetry distance */
  double th_;                        /** odemetry distance */
  double vx_;                        /** velocity X */
  double vth_;                       /** velocity theta */
  
protected:
  /** Called when a new velocity command is published and sends the new velocity to the base motors
   * @param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
   * @note Since the base is nonholonomic, only linear velocities in the x direction and angular velocities around the z direction are supported.
   */
  void velocityCallback(const zuros_motor_transformation::differential::ConstPtr &msg);

  /** Called when a new emergency status message is published
  * @param msg Pointer to std_msgs:Bool message, containing the current status of the emergency stop.
  */
  void emergencyCallback(const std_msgs::Bool::ConstPtr & msg);

  /** Reads the current state of the wheels and publishes this to the topic */
  void odometryPublish();

public:
  /** Default constuctor */
  DPR2Base(ros::NodeHandle nh, std::string config_file_location)
  {
    nh_ = nh;
    init(config_file_location);
  }

  /** Destructor. Deletes the left and right motor object, the config files for both motors, the serial port and shuts the node down */
  ~DPR2Base()
  {
    delete left_motor_;
    delete right_motor_;
    delete config_left_motor_;
    delete config_right_motor_;
    delete serial_port_;
    nh_.shutdown();
  }

  /** Init the motors */
  void init(std::string config_file_location);

  /** Own implementation for ros::spin */
  void spin();
};

#endif /* __DPR2_BASE_H */

