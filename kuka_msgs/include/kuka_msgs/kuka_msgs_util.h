/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021 Kuka Robotics Corp
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
 *   * Neither the name of the Kuka Robotics Corp or Kuka GMBH,
 *     nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
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
 *********************************************************************/

/*
 * Author: Pat Duda <Pat.Duda@kuka.com>
 */
 
#ifndef KUKA_MSGS_
#define KUKA_MSGS_

 
#include <exception>

// ROS
#include <ros/ros.h>

// ROS messages
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Duration.h>
#include <kuka_msgs/ControlType.h>
#include <kuka_msgs/ControlState.h>

// ROS services
#include <std_srvs/Empty.h>
#include <industrial_msgs/StartMotion.h>
#include <industrial_msgs/StopMotion.h>
#include <industrial_msgs/SetDrivePower.h>
#include <industrial_msgs/GetRobotInfo.h>
#include <kuka_msgs/RequestControlType.h>
#include <kuka_msgs/GetControlType.h>

// Timers
#include <chrono>

namespace kuka_msgs
{

class KukaMsgsUtil
{

public:
    KukaMsgsUtil();
    ~KukaMsgsUtil();

    kuka_msgs::ControlType getControlType();
    industrial_msgs::ServiceReturnCode requestControlType(kuka_msgs::ControlType requestedType);

private:
  // ROS node handle
  ros::NodeHandle nh_;
  std::string log_name_;

  ros::ServiceClient client_start_motion_;
  ros::ServiceClient client_stop_motion_;
  ros::ServiceClient client_reset_;
  ros::ServiceClient client_drive_power_on_;
  ros::ServiceClient client_drive_power_off_;
  ros::ServiceClient client_get_robot_info_;
  ros::ServiceClient client_req_control_type_;
  ros::ServiceClient client_get_control_type_;



};
} // namespace kuka_msgs

#endif

