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

#ifndef __MOTOR_TRANSFORMATION_DIFFERENTIAL_H
#define __MOTOR_TRANSFORMATION_DIFFERENTIAL_H

/** @brief Node for transforming the commands coming form zuros_movement to motor commands.
  @author Robert Jacobs
  @date Mar, 12 - 2014
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "zuros_motor_transformation/differential.h"

/// Basic DPR2 base controller class
/**
 * This class reads the following values from the parameter server:
 * \param ~/motor_port The topic name of the \c shared_serial node used for communication with the motor
 * \param ~/motor_config The name of the motor configuration XML file
 * \param ~/wheel_diameter The diameter of the wheels in [m]
 * \param ~/wheel_base The distance between the wheels in [m]
 */
class MotorTransformationDifferential
{
protected:
  ros::NodeHandle nh_;                    /** The ROS nodehandle */
  ros::Subscriber movement_sub_;          /** Subscriber for velocity topic */
  ros::Publisher publisher_differential_; /** Publisher for differntial drive commands */
  double wheel_diameter_, wheel_base_;    /** The platform wheel diameter and wheel base */
  
protected:
  /** Called when a new velocity command is published and sends the new velocity to the base motors
   * @param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
   * @note Since the base is nonholonomic, only linear velocities in the x direction and angular velocities around the z direction are supported.
   */
  void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg);

public:
  /** Default constuctor */
  MotorTransformationDifferential(ros::NodeHandle nh, float wheel_diameter, float wheel_base)
  {
    nh_ = nh;
    wheel_diameter_ = wheel_diameter;
    wheel_base_ = wheel_base;
  }

  /** Destructor. Deletes the left and right motor object, the config files for both motors, the serial port and shuts the node down */
  ~MotorTransformationDifferential()
  {
    nh_.shutdown();
  }

  void init();
};

#endif /* __MOTOR_TRANSFORMATION_DIFFERENTIAL_H */

