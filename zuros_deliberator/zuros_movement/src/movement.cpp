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

#include "zuros_movement/movement.h"
#include <sstream>

/** Constructor
*/
Movement::Movement(ros::NodeHandle nh)
{
  nh_ = nh;
}

/** Initializer
*/ 
void Movement::init()
{
  // Create a publisher
  publisher_cmd_vel_mov_ = nh_.advertise<geometry_msgs::Twist>("/movement", 100);
  
  // Subscribe to topics
  subscriber_cmd_vel_ = nh_.subscribe("/cmd_vel", 100, &Movement::callback_cmd_vel, this);
  subscriber_joy_ = nh_.subscribe("/joy", 100, &Movement::callback_joy, this);
  
  // Set initial parameters
  joystick_override_ = false;
  joystick_override_was_active_ = false;
  ROS_INFO("MOTOR INIT DONE");
}

/** Own implementation of ROS::Spin
*/ 
void Movement::spin()
{  
  while (ros::ok())
  {
    // Joystick override active?
    if(joystick_override_)
    {
      publisher_cmd_vel_mov_.publish(message_);
    }

    // The joystick was active in the past, just to be sure set everythin to 0, otherwise the robot might keep moving after releasing user override button
    else if(joystick_override_was_active_)
    {
      joystick_override_was_active_ = false;
      message_.linear.x = 0;
      message_.linear.y = 0;
      message_.linear.z = 0;

      message_.angular.x = 0;
      message_.angular.y = 0;
      message_.angular.z = 0;

      // Publish to topic
      publisher_cmd_vel_mov_.publish(message_);
    }
    // Make sure that everything is being parsed
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  // After ending loop, send a motor stop command
  message_.linear.x = 0;
  message_.linear.y = 0;
  message_.linear.z = 0;

  message_.angular.x = 0;
  message_.angular.y = 0;
  message_.angular.z = 0;

  publisher_cmd_vel_mov_.publish(message_);
}

/** Callback function for the cmd_vel topic
*   @param msg The message on the topic
*/ 
void Movement::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
  // If there is a joystick override, it would not make sense to send cmd_vel data on the movement topic
  if(!joystick_override_)
  {
    // Publish
    publisher_cmd_vel_mov_.publish(msg);
  }
}

/** Callback function for the joy topic
*   @param msg The message on the topic
*/ 
void Movement::callbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  // User override pressed and not in user override mode will print out the message
  if(msg->buttons[5] == 1 && !joystick_override_)
  {
    ROS_INFO("USER OVERRIDE ACTIVE");
    joystick_override_ = true;
    joystick_override_was_active_ = true;
  }

  // User override released and in user override mode will print out the message
  else if(msg->buttons[5] == 0 && joystick_override_ == true)
  {
    ROS_INFO("USER OVERRIDE RELEASED");  
    joystick_override_ = false;
  }

  // Read the joystick values
  if(joystick_override_)
  {
    // Linear joystick (left joystick) 0?
    if(msg->axes[1] == 0 || msg->axes[1] == -0)
          {
      message_.linear.x = 0;
      message_.linear.x = (0);
          }
    
    // Not 0, use the joystick value as the new value
    else
    {
      message_.linear.x = msg->axes[1];
    }

    // Angular joystick (right joystick) 0?
    if(msg->axes[3] == 0 || msg->axes[3] == -0)
        {
      message_.angular.z = 0;
      message_.angular.z = (0);
        }
    
    // Not 0, use the joystick value as the new value
    else
    {
      message_.angular.z = msg->axes[3];
    }
  }
}
