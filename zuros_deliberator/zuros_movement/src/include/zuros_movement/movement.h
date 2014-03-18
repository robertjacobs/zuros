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

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/callback_queue.h>

class Movement
{
  public:
    Movement(ros::NodeHandle nh);
    void init();
    void spin();
    void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
    void callback_joy(const sensor_msgs::Joy::ConstPtr& msg);
  private:
    bool joystick_override_;
    bool joystick_override_was_active_;
  protected:
    ros::NodeHandle nh_;
    ros::Publisher publisher_cmd_vel_mov_;
    ros::Subscriber subscriber_cmd_vel_;
    ros::Subscriber subscriber_joy_;
    geometry_msgs::Twist message_;
};

#endif /* MOVEMENT_H_ */
