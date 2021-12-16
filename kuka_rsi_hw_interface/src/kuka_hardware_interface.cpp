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
  , rsi_digital_outputs(16)
  , ipoc_(0)
  , n_dof_(6)
  , digital_output_state_(16)
  , digital_output_command_(16)
  , digital_input_state_(16)

{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);
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
  // in_buffer_.resize(1024);
  // recv is a blocking action if the buffer is empty
  if (server_->recv(in_buffer_) <= 0)
  {
    ROS_FATAL("Failed to read state from robot!");
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
  }
  for (std::size_t i = 0; i < digital_input_state_.size(); ++i)
  {
    digital_input_state_[i] = rsi_state_.digital_inputs[i] == 1 ?
                                  hardware_state_command_interfaces::DigitalIOStateHandle::State::HIGH :
                                  hardware_state_command_interfaces::DigitalIOStateHandle::State::LOW;
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
  for (std::size_t i = 0; i < digital_input_state_.size(); ++i)
  {
    digital_input_state_[i] = rsi_state_.digital_inputs[i] == 1 ?
                                  hardware_state_command_interfaces::DigitalIOStateHandle::State::HIGH :
                                  hardware_state_command_interfaces::DigitalIOStateHandle::State::LOW;
  }
  for (std::size_t i = 0; i < digital_output_state_.size(); ++i)
  {
    digital_output_state_[i] = digital_output_command_[i];
  }
  ipoc_ = rsi_state_.ipoc;

  return true;
}

void KukaHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // out_buffer_.resize(1024);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  for (std::size_t i = 0; i < digital_output_command_.size(); ++i)
  {
    rsi_digital_outputs[i] =
        digital_output_command_[i] == hardware_state_command_interfaces::DigitalIOStateHandle::State::HIGH ? 1 : 0;
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

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, rsi_digital_outputs, ipoc_).xml_doc;
  // send is a blocking action if the buffer is full
  if (server_->send(out_buffer_) <= 0)
  {
    ROS_FATAL("Failed to write state to robot!");
  }

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
  if (robot_hw_nh.hasParam("joints"))

  {
    XmlRpc::XmlRpcValue joint_list;
    robot_hw_nh.getParam("joints", joint_list);
    ROS_INFO_STREAM_NAMED("hardware_interface", joint_list);
    if (joint_list.size() != n_dof_)
    {
      ROS_ERROR_STREAM("Number of provided joints are not equal to 6.");
    }

    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      joint_names_[i] = (std::string)(joint_list[i]["name"]);
    }

    // Create ros_control interfaces
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      // Create joint state interface for all joints
      joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

      // Create joint position control interface
      // position_joint_interface_.registerHandle(hardware_interface::JointHandle(
      //    joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));

      // Create joint velocity control interface
      position_velocity_joint_interface_.registerHandle(hardware_interface::PosVelJointHandle(
          joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i], &joint_velocity_command_[i]));
    }

    // Register interfaces
    registerInterface(&joint_state_interface_);
    // registerInterface(&position_joint_interface_);
    registerInterface(&position_velocity_joint_interface_);
  }
  else
  {
    ROS_ERROR_STREAM("joints not found in parameters");
    return false;
  }
  if (robot_hw_nh.hasParam("digital_outputs"))
  {
    XmlRpc::XmlRpcValue digital_output_list;
    robot_hw_nh.getParam("digital_outputs", digital_output_list);
    ROS_INFO_STREAM_NAMED("hardware_interfac", digital_output_list);

    size_t digital_output_size = digital_output_list.size();
    digital_outputs_.resize(digital_output_size);

    for (std::size_t i = 0; i < digital_output_size; ++i)
    {
      std::map<std::string, std::string> digital_output_info;

      digital_output_info["name"] = (std::string)(digital_output_list[i]["name"]);
      digital_output_info["output"] = (std::string)(digital_output_list[i]["output"]);
      digital_output_info["bit_position"] = std::to_string((int)(digital_output_list[i]["bit_position"]));

      digital_outputs_[i] = digital_output_info;
    }
    // Create ros_control interfaces
    for (std::size_t i = 0; i < digital_output_size; ++i)
    {
      auto digital_output_name = digital_outputs_[i]["name"];
      auto digital_output_type_output = digital_outputs_[i]["output"];
      auto digital_output_bit_position = std::stoi(digital_outputs_[i]["bit_position"]);

      if (digital_output_bit_position > 16)
      {
        ROS_ERROR_STREAM_NAMED("hardware_interfac", "Output bit position should be between 0-15");
        return false;
      }
      digital_output_command_[digital_output_bit_position] =
          hardware_state_command_interfaces::DigitalIOStateHandle::State::UNDEFINED;
      digital_output_state_[digital_output_bit_position] =
          hardware_state_command_interfaces::DigitalIOStateHandle::State::UNDEFINED;

      // Create digital output state interface for all digital outputs
      digital_output_state_interface_.registerHandle(hardware_state_command_interfaces::DigitalIOStateHandle(
          digital_output_name, &(digital_output_state_[digital_output_bit_position])));

      // Create digital output command control interface
      digital_output_command_interface_.registerHandle(hardware_state_command_interfaces::DigitalOutputHandle(
          digital_output_state_interface_.getHandle(digital_output_name),
          &(digital_output_command_[digital_output_bit_position])));
    }

    // Register digital output interfaces
    registerInterface(&digital_output_state_interface_);
    registerInterface(&digital_output_command_interface_);
    ROS_INFO_STREAM_NAMED("hardware_interface", "digital_outputs_ init done");
  }

  if (robot_hw_nh.hasParam("digital_inputs"))
  {
    XmlRpc::XmlRpcValue digital_input_list;
    robot_hw_nh.getParam("digital_inputs", digital_input_list);
    ROS_INFO_STREAM_NAMED("hardware_interface", digital_input_list);

    size_t digital_input_size = digital_input_list.size();
    digital_inputs_.resize(digital_input_size);

    for (std::size_t i = 0; i < digital_input_size; ++i)
    {
      std::map<std::string, std::string> digital_input_info;

      digital_input_info["name"] = (std::string)(digital_input_list[i]["name"]);
      digital_input_info["input"] = (std::string)(digital_input_list[i]["input"]);
      digital_input_info["bit_position"] = std::to_string((int)(digital_input_list[i]["bit_position"]));

      digital_inputs_[i] = digital_input_info;
    }

    // Create ros_control interfaces
    for (std::size_t i = 0; i < digital_input_size; ++i)
    {
      auto digital_input_name = digital_inputs_[i]["name"];
      auto digital_input_type_input = digital_inputs_[i]["input"];
      auto digital_input_bit_position = std::stoi(digital_inputs_[i]["bit_position"]);

      if (digital_input_bit_position > 16)
      {
        ROS_ERROR_STREAM_NAMED("hardware_interfac", "Input bit position should be between 0-15");
        return false;
      }

      digital_input_state_[digital_input_bit_position] =
          hardware_state_command_interfaces::DigitalIOStateHandle::State::UNDEFINED;

      // Create digital input state interface for all digital outputs
      digital_input_state_interface_.registerHandle(hardware_state_command_interfaces::DigitalIOStateHandle(
          digital_input_name, &(digital_input_state_[digital_input_bit_position])));
    }

    // Register digital input interfaces
    registerInterface(&digital_input_state_interface_);

    ROS_INFO_STREAM_NAMED("hardware_interfac", "digital_inputs_ init done");
  }
  configure();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_rsi_hardware_interface");
  // start();
  return true;
}
}  // namespace kuka_rsi_hw_interface

PLUGINLIB_EXPORT_CLASS(kuka_rsi_hw_interface::KukaHardwareInterface, hardware_interface::RobotHW)
