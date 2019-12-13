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
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

#include <stdexcept>

namespace kuka_rsi_hw_interface
{
KukaHardwareInterface::KukaHardwareInterface()
  : joint_position_(6, 0.0)
  , joint_velocity_(6, 0.0)
  , joint_effort_(6, 0.0)
  , joint_position_command_(6, 0.0)
  , joint_velocity_command_(6, 0.0)
  , joint_effort_command_(6, 0.0)
  , joint_names_(6)
  , rsi_initial_joint_positions_(6, 0.0)
  , rsi_joint_position_corrections_(6, 0.0)
  , ipoc_(0)
  , n_dof_(6)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'controller_joint_names' on the parameter server.");
  }

  // Create ros_control interfaces
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                               &joint_velocity_[i], &joint_effort_[i]));

    // Create joint position control interface
    position_joint_interface_.registerHandle(hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_rsi_hardware_interface");
}

KukaHardwareInterface::~KukaHardwareInterface()
{
}

void KukaHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  if (first_time_)
  {
    start();
    first_time_ = false;
  }
  in_buffer_.resize(1024);

  server_->recv(in_buffer_);

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
  }
  ipoc_ = rsi_state_.ipoc;
  if (rt_rsi_kuka_to_pc_pub_->trylock())
  {
    rt_rsi_kuka_to_pc_pub_->msg_.time = time;
    rt_rsi_kuka_to_pc_pub_->msg_.period = period;
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      rt_rsi_kuka_to_pc_pub_->msg_.AIPos_joint_actual_position[i] = rsi_state_.positions[i];
      rt_rsi_kuka_to_pc_pub_->msg_.ASPos_joint_initial_position[i] = rsi_state_.initial_positions[i];
      rt_rsi_kuka_to_pc_pub_->msg_.RIst_cartesian_actual_position[i] = rsi_state_.cart_position[i];
      rt_rsi_kuka_to_pc_pub_->msg_.RSol_cartesian_initial_position[i] = rsi_state_.initial_cart_position[i];
    }

    rt_rsi_kuka_to_pc_pub_->msg_.ipoc = rsi_state_.ipoc;
    rt_rsi_kuka_to_pc_pub_->unlockAndPublish();
  }
}
bool KukaHardwareInterface::read()
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }

  rsi_state_ = RSIState(in_buffer_);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
  }
  ipoc_ = rsi_state_.ipoc;

  return true;
}

void KukaHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  if (rt_rsi_pc_to_kuka_pub_->trylock())
  {
    rt_rsi_pc_to_kuka_pub_->msg_.time = time;
    rt_rsi_pc_to_kuka_pub_->msg_.period = period;
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      rt_rsi_pc_to_kuka_pub_->msg_.joint_target_position[i] = RAD2DEG * joint_position_command_[i];
      rt_rsi_pc_to_kuka_pub_->msg_.AK_joint_correction[i] = rsi_joint_position_corrections_[i];
      rt_rsi_pc_to_kuka_pub_->msg_.ASPos_joint_initial_position[i] = rsi_state_.initial_positions[i];
    }

    rt_rsi_pc_to_kuka_pub_->msg_.ipoc = rsi_state_.ipoc;
    rt_rsi_pc_to_kuka_pub_->unlockAndPublish();
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);

  // return true;
}

void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
  }
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  // server_->set_timeout(1000);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");
}

void KukaHardwareInterface::configure()
{
  const std::string param_addr = "rsi/listen_address";
  const std::string param_port = "rsi/listen_port";

  if (nh_.getParam(param_addr, local_host_) && nh_.getParam(param_port, local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    std::string msg = "Failed to get RSI listen address or listen port from"
                      " parameter server (looking for '" +
                      param_addr + "' and '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
  rt_rsi_kuka_to_pc_pub_.reset(
      new realtime_tools::RealtimePublisher<kuka_rsi_hw_interface_msgs::RSIReceived>(nh_, "rt_rsi_kuka_to_pc_pub", 3));
  rt_rsi_pc_to_kuka_pub_.reset(
      new realtime_tools::RealtimePublisher<kuka_rsi_hw_interface_msgs::RSISent>(nh_, "rt_rsi_pc_to_kuka_pub", 3));
}

bool KukaHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  configure();
  // start();
  return true;
}
}  // namespace kuka_rsi_hw_interface

PLUGINLIB_EXPORT_CLASS(kuka_rsi_hw_interface::KukaHardwareInterface, hardware_interface::RobotHW)
