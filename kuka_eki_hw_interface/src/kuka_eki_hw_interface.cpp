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


#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <angles/angles.h>

#include <tinyxml.h>

#include <kuka_eki_hw_interface/kuka_eki_hw_interface.h>


namespace kuka_eki_hw_interface
{

KukaEkiHardwareInterface::KukaEkiHardwareInterface() :
    n_dof_(6), joint_position_(6, 0.0), joint_velocity_(6, 0.0), joint_effort_(6, 0.0),
    joint_position_command_(6, 0.0), joint_names_(6),
    eki_server_socket_(ios_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
{
  // Get controller joint names
  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter 'controller_joint_names' on the parameter server.");
  }

  // Create ros_control interfaces (joint state and position joint for all dof's)
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Joint state interface
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                             &joint_effort_[i]));

    // Joint position control interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO_STREAM_NAMED("kukw_eki_hw_interface", "Loaded Kuka EKI hardware interface");
}


KukaEkiHardwareInterface::~KukaEkiHardwareInterface()
{

}


bool KukaEkiHardwareInterface::eki_read_state(std::vector<double> &joint_position,
                                                 std::vector<double> &joint_velocity,
                                                 std::vector<double> &joint_effort)
{
  static boost::array<char, 2048> in_buffer;

  // Read socket buffer
  size_t len = eki_server_socket_.receive_from(boost::asio::buffer(in_buffer), eki_server_endpoint_);

  // Update joint positions from XML packet (if received)
  if (len == 0)
    return false;

  // Parse XML
  TiXmlDocument xml_in;
  in_buffer[len] = '\0';  // null-terminate data buffer for parsing (expects c-string)
  xml_in.Parse(in_buffer.data());
  TiXmlElement* robot_state = xml_in.FirstChildElement("RobotState");
  TiXmlElement* pos = robot_state->FirstChildElement("Pos");
  TiXmlElement* vel = robot_state->FirstChildElement("Vel");
  TiXmlElement* eff = robot_state->FirstChildElement("Eff");

  // Extract axis positions
  double joint_pos;  // [deg]
  double joint_vel;  // [%max]
  double joint_eff;  // [Nm]
  char axis_name[] = "A1";
  for (int i = 0; i < n_dof_; ++i)
  {
    pos->Attribute(axis_name, &joint_pos);
    joint_position[i] = angles::from_degrees(joint_pos);  // convert deg to rad
    vel->Attribute(axis_name, &joint_vel);
    joint_velocity[i] = joint_vel;
    eff->Attribute(axis_name, &joint_eff);
    joint_effort[i] = joint_eff;
    axis_name[1]++;
  }
  ROS_INFO_STREAM(joint_effort[0]);

  return true;
}


bool KukaEkiHardwareInterface::eki_write_command(const std::vector<double> &joint_position_command)
{
  TiXmlDocument xml_out;
  TiXmlElement* robot_command = new TiXmlElement("RobotCommand");
  TiXmlElement* pos = new TiXmlElement("Pos");
  TiXmlText* empty_text = new TiXmlText("");
  robot_command->LinkEndChild(pos);
  pos->LinkEndChild(empty_text);   // force <Pos></Pos> format (vs <Pos />)
  char axis_name[] = "A1";
  for (int i = 0; i < n_dof_; ++i)
  {
    pos->SetAttribute(axis_name, std::to_string(angles::to_degrees(joint_position_command[i])).c_str());
    axis_name[1]++;
  }
  xml_out.LinkEndChild(robot_command);

  TiXmlPrinter xml_printer;
  xml_printer.SetStreamPrinting();  // no linebreaks
  xml_out.Accept(&xml_printer);

  size_t len = eki_server_socket_.send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()),
                                          eki_server_endpoint_);

  return true;
}


void KukaEkiHardwareInterface::start()
{
  ROS_INFO_NAMED("kuka_eki_hw_interface", "Starting Kuka EKI hardware interface...");

  // Start client
  ROS_INFO_NAMED("kuka_eki_hw_interface", "... connecting to robot's EKI server...");
  boost::asio::ip::udp::resolver resolver(ios_);
  eki_server_endpoint_ = *resolver.resolve({boost::asio::ip::udp::v4(), eki_server_address_, eki_server_port_});
  boost::array<char, 1> ini_buf = { 0 };
  eki_server_socket_.send_to(boost::asio::buffer(ini_buf), eki_server_endpoint_);  // initiate contact to start server

  // Initialize joint_position_command_ from initial robot state (avoid bad (null) commands before controllers come up)
  while (!eki_read_state(joint_position_, joint_velocity_, joint_effort_));
  joint_position_command_ = joint_position_;

  ROS_INFO_NAMED("kuka_eki_hw_interface", "... done. EKI hardware interface started!");
}

void KukaEkiHardwareInterface::configure()
{
  const std::string param_addr = "eki/robot_address";
  const std::string param_port = "eki/robot_port";

  if (nh_.getParam(param_addr, eki_server_address_) &&
      nh_.getParam(param_port, eki_server_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_eki_hw_interface", "Configuring Kuka EKI hardware interface on: "
                          << eki_server_address_ << ", " << eki_server_port_);
  }
  else
  {
    std::string msg = "Failed to get EKI address/port from parameter server (looking for '" + param_addr +
                      "', '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
}


void KukaEkiHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
{
  eki_read_state(joint_position_, joint_velocity_, joint_effort_);
}


void KukaEkiHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
{
  eki_write_command(joint_position_command_);
}

} // namespace kuka_eki_hw_interface
