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

#include "zuros_motor_transformation/motor_transformation_differential.h"
#include "zuros_motor_transformation/differential.h"

/** Constructor
*/ 
void MotorTransformationDifferential::init()
{
  ROS_INFO("Initializing motor_transformation");

  // Subscribe to movement topic
  movement_sub_ = nh_.subscribe("/movement", 1, &MotorTransformationDifferential::velocityCallback, this);

  // Create differential publisher
  publisher_differential_ = nh_.advertise<zuros_motor_transformation::differential>("/motor_transformation_differential", 1);
}

/** Called when a new velocity command is published and sends the new velocity to the base motors
* @param msg Pointer to geometry_msgs/Twist message, containing the linear and angular velocities, in [m/s] and [rad/s] respectively.
* @note Since the base is nonholonomic, only linear velocities in the x direction and angular velocities around the z direction are supported.
*/
void MotorTransformationDifferential::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  try
  {  
    // Base is nonholonomic, warn if sent a command we can't execute
    if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y)
    {
      ROS_WARN("It is not possible for me to translate this command to the robot - maybe you set the base type in navigation config to holonomic instead of nonholonomic or vice versa?");
      return;
    }

	// Check not a number
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

    //publish
    zuros_motor_transformation::differential msg;
    msg.left_motor_speed = vel_left;
    msg.right_motor_speed = vel_right;
    publisher_differential_.publish(msg);
  }
 
  catch (int e)
  {
    ROS_INFO("[motor_transformation] : An exception occurred in velocity callback: %i", e);
  }
}
