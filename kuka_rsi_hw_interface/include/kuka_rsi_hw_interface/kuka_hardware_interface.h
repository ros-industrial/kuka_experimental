/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
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
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
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
 * Author: Lars Tingelstad
 */

#ifndef KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_
#define KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_

// STL
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>


// ros_control
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Timers
#include <chrono>

// UDP server
#include <kuka_rsi_hw_interface/udp_server.h>

// RSI
#include <kuka_rsi_hw_interface/rsi_state.h>
#include <kuka_rsi_hw_interface/rsi_command.h>

// SRV
#include <kuka_rsi_hw_interface/write_8_outputs.h>
#include <kuka_rsi_hw_interface/request_command.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <fdcc/fdcc.h>

namespace kuka_rsi_hw_interface
{

static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

class KukaHardwareInterface : public hardware_interface::RobotHW
{

private:

  // ROS node handle
  ros::NodeHandle nh_;

  unsigned int n_dof_;

  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  std::vector<bool> digital_output_;

  // RSI
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::vector<double> rsi_initial_joint_positions_;
  std::vector<double> rsi_joint_position_corrections_;
  unsigned long long ipoc_;

  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > rt_rsi_pub_;

  std::unique_ptr<UDPServer> server_;
  std::string local_host_;
  int local_port_;
  std::string remote_host_;
  std::string remote_port_;
  std::string in_buffer_;
  std::string out_buffer_;

  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;


  // Testing
  ros::Publisher JointCommandPub;
  ros::Publisher DigitalInputsStatePub;

  ros::Subscriber JointCmdSub;

  ros::ServiceClient RSI_cmdSrv;

  int dummySrvCnt;




public:

  KukaHardwareInterface();
  ~KukaHardwareInterface();

  //Test section for adding digital output control
  void PublishDigitalInputs(std::vector<double> &digital_inputs_state);
  void JointCmdCallback    (const trajectory_msgs::JointTrajectory &msg);
  bool write_8_digital_outputs(kuka_rsi_hw_interface::write_8_outputs::Request &req, kuka_rsi_hw_interface::write_8_outputs::Response &res);

  void start();
  void configure();
  bool read(const ros::Time time, const ros::Duration period);
  bool write(const ros::Time time, const ros::Duration period, std::vector<float> v);

  double joint_position[6];
  int seq_temp;
  int seq_old;

  
};

} // namespace kuka_rsi_hw_interface

#endif
