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

#include <kuka_rsi_hw_interface/kuka_position_interface.h>

#include <stdexcept>

#include <pthread.h>
#include <sched.h>

namespace kuka_rsi_hw_interface
{

KukaPositionInterface::KukaPositionInterface() :
    joint_position_(6, 0.0), joint_velocity_(6, 0.0), joint_effort_(6, 0.0), joint_position_command_(6, 0.0), joint_velocity_command_(
        6, 0.0), joint_effort_command_(6, 0.0), joint_names_(6), rsi_initial_joint_positions_(6, 0.0), rsi_joint_position_corrections_(
        6, 0.0), ipoc_(0), n_dof_(6), digital_output_(8,false)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  DigitalInputsStatePub = nh_.advertise<std_msgs::Float64MultiArray>("/rsi_digital_inputs_state", 1);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'controller_joint_names' on the parameter server.");
  }

  //Create ros_control interfaces
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                             &joint_effort_[i]));

    // Create joint position control interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO_STREAM_NAMED("position_interface", "Loaded kuka_rsi_position_interface");
}

KukaPositionInterface::~KukaPositionInterface()
{

}


void KukaPositionInterface::PublishDigitalInputs(std::vector<double> &digital_inputs_state)
{

  std_msgs::Float64MultiArray msg;

  msg.data.push_back(digital_inputs_state[0]);
  msg.data.push_back(digital_inputs_state[1]);
  msg.data.push_back(digital_inputs_state[2]);
  msg.data.push_back(digital_inputs_state[3]);
  msg.data.push_back(digital_inputs_state[4]);
  msg.data.push_back(digital_inputs_state[5]);
  msg.data.push_back(digital_inputs_state[6]);
  msg.data.push_back(digital_inputs_state[7]);

  this->DigitalInputsStatePub.publish(msg);

}

void KukaPositionInterface::JointCmdCallback    (const trajectory_msgs::JointTrajectory &msg)
{

/*
    this->joint_position[0] = msg.points[0].positions[0];
    this->joint_position[1] = msg.points[0].positions[1];
    this->joint_position[2] = msg.points[0].positions[2];
    this->joint_position[3] = msg.points[0].positions[3];
    this->joint_position[4] = msg.points[0].positions[4];
    this->joint_position[5] = msg.points[0].positions[5];

    this->seq_temp = msg.header.seq;
*/
/*    this->joint_position[1] = msg.data[1];
    this->joint_position[2] = msg.data[2];
    this->joint_position[3] = msg.data[3];
    this->joint_position[4] = msg.data[4];
    this->joint_position[5] = msg.data[5];
*/
}


bool KukaPositionInterface::write_8_digital_outputs(kuka_rsi_hw_interface::write_8_outputs::Request &req, kuka_rsi_hw_interface::write_8_outputs::Response &res){

  /*
   digital_output_.clear();
   digital_output_.push_back(req.out1);
   digital_output_.push_back(req.out2);
   digital_output_.push_back(req.out3);
   digital_output_.push_back(req.out4);
   digital_output_.push_back(req.out5);
   digital_output_.push_back(req.out6);
   digital_output_.push_back(req.out7);
   digital_output_.push_back(req.out8);
  */

  if (req.ID >= 0 && req.ID < 8)
    digital_output_[req.ID] = req.value;

   return true;
}

bool KukaPositionInterface::read(const ros::Time time, const ros::Duration period)
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }

  if (rt_rsi_pub_->trylock()){
    rt_rsi_pub_->msg_.data = in_buffer_;
    rt_rsi_pub_->unlockAndPublish();
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    // update digital input states
    this->PublishDigitalInputs(rsi_state_.digital_inputs);
  }
  ipoc_ = rsi_state_.ipoc;

  return true;
}

bool KukaPositionInterface::write(const ros::Time time, const ros::Duration period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, digital_output_, ipoc_).xml_doc;
  server_->send(out_buffer_);

  return true;
}

void KukaPositionInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_position_interface", "Waiting for robot!");

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
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, digital_output_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  ROS_INFO_STREAM_NAMED("kuka_position_interface", "Got connection from robot");

}

void KukaPositionInterface::configure()
{
  const std::string param_addr = "rsi/listen_address";
  const std::string param_port = "rsi/listen_port";

  if (nh_.getParam(param_addr, local_host_) && nh_.getParam(param_port, local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_position_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    std::string msg = "Failed to get RSI listen address or listen port from"
    " parameter server (looking for '" + param_addr + "' and '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
  rt_rsi_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc", 3));
}

} // namespace kuka_rsi_position_interface