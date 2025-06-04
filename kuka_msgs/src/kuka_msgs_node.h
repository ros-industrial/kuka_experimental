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



#ifndef KUKA_MSGS_NODE_
#define KUKA_MSGS_NODE_
#define TEST_COMM_TIMING

// STL
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>



namespace kuka_msgs
{

class KukaMsgsNode
{

public:

  KukaMsgsNode();
  ~KukaMsgsNode();







protected:
//  virtual bool startMotionCB(industrial_msgs::StartMotion::Request &request,
//                            industrial_msgs::StartMotion::Response &response);

//  virtual bool stopMotionCB(industrial_msgs::StopMotion::Request &request,
//                            industrial_msgs::StopMotion::Response &response);

//  virtual bool resetCB(std_srvs::Empty::Request &request,
//                       std_srvs::Empty::Response &response);

//  virtual bool setDrivePowerOnCB(industrial_msgs::SetDrivePower::Request &request,
//                                 industrial_msgs::SetDrivePower::Response &response);

//  virtual bool setDrivePowerOffCB(industrial_msgs::SetDrivePower::Request &request,
//                                 industrial_msgs::SetDrivePower::Response &response);

//  virtual bool getRobotInfoCB(industrial_msgs::GetRobotInfo::Request &request,
//                            industrial_msgs::GetRobotInfo::Response &response);

//  virtual bool getControlTypeCB(kuka_msgs::GetControlType::Request &request,
//                                kuka_msgs::GetControlType::Response &response);

//  virtual bool requestControlTypeCB(kuka_msgs::RequestControlType::Request &request,
//                                    kuka_msgs::RequestControlType::Response &response);



};

} /// namespace kuka_msgs

#endif
