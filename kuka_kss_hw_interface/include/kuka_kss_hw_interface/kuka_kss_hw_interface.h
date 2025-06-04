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
 * Based on kuka_rsi_hw_interface from: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */



#ifndef KUKA_KSS_HARDWARE_INTERFACE_
#define KUKA_KSS_HARDWARE_INTERFACE_
#define TEST_COMM_TIMING

// Communication protocol: Must match ROS_EKI_COMM_MODE in robot kuka_ros_driver.dat
// Select the EKI protocol - Comment out the following line for binary
//#define EKI_COMM_XML
#ifndef EKI_COMM_XML
#define EKI_COMM_BIN
#endif

// STL
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
// ros_control
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_clock.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

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

#include <kuka_kss_hw_interface/kuka_robot_state.h>
#include <kuka_kss_hw_interface/comm_link_tcp_client.h>
#include <kuka_kss_hw_interface/comm_link_udp_server.h>
#include <kuka_kss_hw_interface/kuka_comm_handler_rsi.h>
#ifdef EKI_COMM_XML
#include <kuka_kss_hw_interface/kuka_comm_handler_eki.h>
#elif defined(EKI_COMM_BIN)
#include <kuka_kss_hw_interface/kuka_comm_handler_ekiBIN.h>
#endif

namespace kuka_kss_hw_interface
{
static const std::string RSI_INTERFACE_VERSION = "V2001ROS";
static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

class KukaKssHardwareInterface : public hardware_interface::RobotHW
{

public:

  KukaKssHardwareInterface();
  ~KukaKssHardwareInterface();
  void testTCPcomm();
  void testUDPcomm();

  /**
   * @brief controlLoop - implements the main control loop read-update-write for ROS Control.
   * @param controller_mgr
   */
  void controlLoop(controller_manager::ControllerManager& controller_mgr);
  void controlLoopRSI(controller_manager::ControllerManager& controller_mgr);

  void configure();
  void start();
  void startRSI();
  bool read(const ros::Time time, const ros::Duration period);
  bool readRSI(const ros::Time time, const ros::Duration period);
  bool write(const ros::Time time, const ros::Duration period);
  bool writeRSI(const ros::Time time, const ros::Duration period);

  void debugDumpControllerInfo(const std::list<hardware_interface::ControllerInfo>& ctl_list);

  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list);

  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list);

  RobotHW::SwitchState switchResult() const;


  /** \brief Return (in realtime) the state of the last doSwitch() for a given controller. */
  RobotHW::SwitchState switchResult(const hardware_interface::ControllerInfo& controller) const;

  std::string activeController() const;

  kuka_msgs::ControlState determineRobOpState(kuka_msgs::ControlType ctrl_type, std::string ROScontroller);


protected:
  virtual bool startMotionCB(industrial_msgs::StartMotion::Request &request,
                            industrial_msgs::StartMotion::Response &response);

  virtual bool stopMotionCB(industrial_msgs::StopMotion::Request &request,
                            industrial_msgs::StopMotion::Response &response);

  virtual bool resetCB(std_srvs::Empty::Request &request,
                       std_srvs::Empty::Response &response);

  virtual bool setDrivePowerOnCB(industrial_msgs::SetDrivePower::Request &request,
                                 industrial_msgs::SetDrivePower::Response &response);

  virtual bool setDrivePowerOffCB(industrial_msgs::SetDrivePower::Request &request,
                                 industrial_msgs::SetDrivePower::Response &response);

  virtual bool getRobotInfoCB(industrial_msgs::GetRobotInfo::Request &request,
                            industrial_msgs::GetRobotInfo::Response &response);

  virtual bool getControlTypeCB(kuka_msgs::GetControlType::Request &request,
                                kuka_msgs::GetControlType::Response &response);

  virtual bool requestControlTypeCB(kuka_msgs::RequestControlType::Request &request,
                                    kuka_msgs::RequestControlType::Response &response);


private:
  /**
   * @brief configure_thread_priority - Setup real-time process priority.  Probably not portable outside Linux.
   * @return
   */
  bool configure_thread_priority();

  // ROS node handle
  ros::NodeHandle nh_;
  std::string log_name_;

  int robot_state_id_;
  CommLink_UDPServer udp_server_;
  CommLink_TCPClient tcp_to_robot_;
#ifdef EKI_COMM_XML
  std::unique_ptr<KukaCommHandlerEKI> rob_connection_;
#elif defined(EKI_COMM_BIN)
  std::unique_ptr<KukaCommHandlerEKIBIN> rob_connection_;
#endif
  std::unique_ptr<KukaCommHandlerRSI> rsi_connection_;
  bool rsi_did_startup_;
  ros::Time rsi_remote_start_time_;

  unsigned int ros_dof_;
  std::vector<std::string> joint_names_;
  // Keep list of Controllers
  std::map<std::string, std::string> controller_list_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::PosVelJointInterface posvel_joint_interface_;

  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > rt_rsi_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Int16> > rt_rsi_pub_delay_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Int64> > rt_pub_ipoc_;
  std::unique_ptr<realtime_tools::RealtimePublisher<industrial_msgs::RobotStatus> > rt_pub_robstatus_;
  //std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Int64> > rt_rsi_pub_robtime_;
#ifdef TEST_COMM_TIMING
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Duration> > rt_eki_pub_ipoc_interval_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Duration> > rt_rsi_pub_resp_timeP_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Duration> > rt_rsi_pub_resp_timeU_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Duration> > rt_rsi_pub_resp_timeW_;
#endif

  ros::ServiceServer srv_start_motion_;
  ros::ServiceServer srv_stop_motion_;
  ros::ServiceServer srv_reset_;
  ros::ServiceServer srv_drive_power_on_;
  ros::ServiceServer srv_drive_power_off_;
  ros::ServiceServer srv_get_robot_info_;
  ros::ServiceServer srv_req_control_type_;
  ros::ServiceServer srv_get_control_type_;


#ifdef TEST_COMM_TIMING
  ros::Duration response_timeP_;
  ros::Duration response_timeU_;
  ros::Duration response_timeW_;
#endif




};

} // namespace kuka_kss_hw_interface

#endif
