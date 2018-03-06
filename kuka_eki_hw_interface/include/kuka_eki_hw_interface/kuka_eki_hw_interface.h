/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, 3M
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the copyright holder, nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Brett Hemes (3M) <brhemes@mmm.com>


#ifndef KUKA_EKI_HW_INTERFACE
#define KUKA_EKI_HW_INTERFACE

#include <vector>
#include <string>

#include <boost/asio.hpp>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace kuka_eki_hw_interface
{

class KukaEkiHardwareInterface : public hardware_interface::RobotHW
{
private:
  ros::NodeHandle nh_;

  unsigned int n_dof_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;

  // EKI
  boost::asio::io_service ios_;
  std::string state_server_address_;
  std::string state_server_port_;
  boost::asio::ip::udp::endpoint eki_state_endpoint_;
  boost::asio::ip::udp::socket eki_state_socket_;
  std::string command_server_address_;
  std::string command_server_port_;
  boost::asio::ip::udp::endpoint eki_command_endpoint_;
  boost::asio::ip::udp::socket eki_command_socket_;

  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  // Private socket read/write member functions
  bool socket_read_state(std::vector<double> &joint_position);
  bool socket_write_command(const std::vector<double> &joint_position);

public:
  KukaEkiHardwareInterface();
  ~KukaEkiHardwareInterface();

  void start();
  void configure();
  void read(const ros::Time &time, const ros::Duration &period);
  void write(const ros::Time &time, const ros::Duration &period);
};

} // namespace kuka_eki_hw_interface

#endif  // KUKA_EKI_HW_INTERFACE
