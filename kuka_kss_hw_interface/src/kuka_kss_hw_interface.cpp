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

#include <kuka_kss_hw_interface/kuka_commands.h>
#include <kuka_kss_hw_interface/kuka_robot_state_manager.h>
#include <kuka_kss_hw_interface/kuka_kss_hw_interface.h>
#include <kuka_kss_hw_interface/comm_link_tcp_client.h>
#include <stdexcept>



namespace kuka_kss_hw_interface
{

KukaKssHardwareInterface::KukaKssHardwareInterface() :
    log_name_("kuka_kss_hw_interface"), udp_server_(nh_, log_name_),tcp_to_robot_(nh_, log_name_),
    joint_names_(6), ros_dof_(6), rsi_did_startup_(false)
{
   robot_state_id_ = KukaRobotStateManager::addRobot();
   ROS_INFO_NAMED(log_name_, "Added robot state. ID=%d", robot_state_id_);

  // This can be variable size vector in case externals are used.
  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR_STREAM_NAMED(log_name_, "Cannot find required parameter 'controller_joint_names' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'controller_joint_names' on the parameter server.");
  }
  if (joint_names_.size() < 6)
  {
      ROS_ERROR_STREAM_NAMED(log_name_, "The parameter 'controller_joint_names' contains less than 6 joints. Not supported.");
      throw std::runtime_error("The parameter 'controller_joint_names' contains less than 6 joints. Not supported.");
  }

  // Handle external axes
  if (joint_names_.size() > 12)
  {
      ROS_ERROR_STREAM_NAMED(log_name_, "Parameter 'controller_joint_names' contains more than 12 joints. Extras ignored.");
      ros_dof_ = 12;
      joint_names_.resize(12);
  }
  else
      ros_dof_ = joint_names_.size();

  ROS_INFO_NAMED(log_name_, "controller_joint_names configured for %lu axes.", joint_names_.size());

  //Create ros_control interfaces
  for (std::size_t i = 0; i < ros_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i],
                                             KukaRobotStateManager::getRobot(robot_state_id_).getJointPositionPtr(i),
                                             KukaRobotStateManager::getRobot(robot_state_id_).getJointVelocityPtr(i),
                                             KukaRobotStateManager::getRobot(robot_state_id_).getJointEffortPtr(i)));

    // Create joint position control interface

    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        KukaRobotStateManager::getRobot(robot_state_id_).getCommandJointPositionPtr(i)));


    // Create joint position & velocity control interface
    posvel_joint_interface_.registerHandle(
                hardware_interface::PosVelJointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                      KukaRobotStateManager::getRobot(robot_state_id_).getCommandJointPositionPtr(i),
                                                      KukaRobotStateManager::getRobot(robot_state_id_).getCommandJointVelocityPtr(i)));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&posvel_joint_interface_);
  controller_list_.clear();
  rsi_remote_start_time_ = ros::Time::now();

  ROS_INFO_STREAM_NAMED(log_name_, "Loaded kuka_kss_hardware_interface");

}

KukaKssHardwareInterface::~KukaKssHardwareInterface()
{
}


void KukaKssHardwareInterface::controlLoop(controller_manager::ControllerManager& controller_mgr)
{
    ros::Time timestamp;
    ros::Duration period;
    kuka_msgs::ControlType previous_control_type;
    kuka_msgs::ControlType current_control_type;

    start();
    current_control_type = KukaRobotStateManager::getRobot(robot_state_id_).getSelectedControlType();

    // Initial implementation - EKI & RSI control loop
    while (ros::ok())
    {
        //ROS_INFO_NAMED( log_name_, "Main control loop run 1.");

        {
            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getRobotMutex());
            period = KukaRobotStateManager::getRobot(robot_state_id_).getIpocDuration();
            timestamp = ros::Time::now();
            previous_control_type = current_control_type;
            current_control_type = KukaRobotStateManager::getRobot(robot_state_id_).getSelectedControlType();
        }

        if (current_control_type.control_type != previous_control_type.control_type)
        {
            ROS_INFO_NAMED( log_name_, "New control type=%d", current_control_type.control_type);
        }

        //ROS_INFO_NAMED( log_name_, "Main control loop run 2a. period=%f", period.toSec());
        bool receivedMessage = false;
        switch (current_control_type.control_type)
        {
        case kuka_msgs::ControlType::KUKA_KRL:
            if (rob_connection_->getCommState() != KukaCommHandler::KukaCommState::CONNECTED)
            {
                ROS_WARN_THROTTLE_NAMED(30.0, log_name_, "EKI Server Connection lost.  Attempting to reconnect.");
                rob_connection_->shutdownComm();

                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                start();
                // TODO: Reconnect causes exception: "Transport endpoint is already connected".  Need to add disconnect method to CommunicationLink class (communication_link.h)
            }
            else
            {
                receivedMessage = read(timestamp, period);
            }
            break;
        case kuka_msgs::ControlType::KUKA_RSI4ms:
        case kuka_msgs::ControlType::KUKA_RSI12ms:
            if (!rsi_did_startup_ || (rsi_connection_->getCommState() != KukaCommHandler::KukaCommState::CONNECTED))
            {
                ROS_WARN_THROTTLE_NAMED(30.0, log_name_, "RSI not connected.  Attempting to reconnect.");
                // Make sure RSI program is started on robot -
                bool do_send_start = false;
                if (current_control_type.control_type != previous_control_type.control_type)
                {
                    do_send_start = true;
                }
                // Dont send start more than once every 0.5 sec
                ros::Duration t_since_remote_start;
                t_since_remote_start =  timestamp - rsi_remote_start_time_;
                if (t_since_remote_start.toSec() > 0.5)
                {
                    if (KukaRobotStateManager::getRobot(robot_state_id_).getRobRunState() != KukaRobotState::KukaRunState::Running)
                        do_send_start = true;
                }
                if (do_send_start)
                {
                    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Reset), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
                    rob_connection_->messageSend();
                    std::this_thread::sleep_for(std::chrono::milliseconds(4));

                    rsi_remote_start_time_ = timestamp;
                    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Start), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
                    rob_connection_->messageSend();
                    ROS_INFO_NAMED( log_name_, "RSI Sending START command.");
                    rsi_did_startup_ = false;
                }

                startRSI();
            }
            if (rsi_did_startup_)
            {
                receivedMessage = readRSI(timestamp, period);
            }
            break;
        }

        //ROS_INFO_NAMED( log_name_, "Main control loop run 2b.");
        // TODO - implement timeout for RSI comm to detect lost connection
        if (receivedMessage)
        {
            {
                //ROS_INFO_NAMED( log_name_, "Main control loop run 3.");
                std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getRobotMutex());
                //ROS_INFO_NAMED( log_name_, "Main control loop run 4.");
                controller_mgr.update(timestamp, period);
            }

            // Based on current_control_type, call write() or writeRSI()
            bool write_success = false;
            switch (current_control_type.control_type)
            {
            case kuka_msgs::ControlType::KUKA_KRL:
                write_success = write(timestamp, period);
                break;
            case kuka_msgs::ControlType::KUKA_RSI4ms:
            case kuka_msgs::ControlType::KUKA_RSI12ms:
                write_success = writeRSI(timestamp, period);
                break;
            }
            if (!write_success)
            {
                //TODO - add better fallback.  Switch back to EKI if ROS comm fails?  Switch back to listen & reconnect if no connections.
                ROS_FATAL_NAMED(log_name_, "Failed to write state to robot!");
                //ros::shutdown();
                //ROS_FATAL_NAMED(log_name_, "Shutting down!");
            }

        }
        else
        {
            // Make sure we handle EKI comm link when running in RSI mode - handle Robot Status changes
            if (current_control_type.control_type != kuka_msgs::ControlType::KUKA_KRL)
            {
                read(timestamp, period);

                // Keep the connection alive with a heartbeat every 1.5s
                if (tcp_to_robot_.time_since_last_comm().toSec() > 1.5)
                {
                    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Heartbeat), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
                    rob_connection_->messageSend();
                }

            }

            // Handle publishing ROS robot status changes
            if (KukaRobotStateManager::getRobot(robot_state_id_).statusChanged())
            {
                if (rt_pub_robstatus_->trylock()){
                    std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getStatusMutex());
                    rt_pub_robstatus_->msg_ = KukaRobotStateManager::getRobot(robot_state_id_).getRos_status();
                    rt_pub_robstatus_->unlockAndPublish();
                    //ROS_INFO_NAMED( log_name_, "Main control loop run 5.");
                }
            }

            // sleep 0.5 millisec (probably will use OS minimum slice)
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
}

void KukaKssHardwareInterface::controlLoopRSI(controller_manager::ControllerManager& controller_mgr)
{
    ros::Time timestamp;
    ros::Duration period;

    startRSI();

    // Initial implementation - RSI only control loop
    while (ros::ok())
    {
        timestamp = ros::Time::now();
        period = KukaRobotStateManager::getRobot(robot_state_id_).getIpocDuration();
        bool receivedMessage = readRSI(timestamp, period);

        if (receivedMessage)
        {
            controller_mgr.update(timestamp, period);

            if (!writeRSI(timestamp, period))
            {
                ROS_FATAL_NAMED(log_name_, "Failed to write state to robot. Shutting down!");
                ros::shutdown();
            }
        }
        else
        {
            // sleep 0.5 millisec (probably will use OS minimum slice)
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
}



bool KukaKssHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
    bool received_msg = rob_connection_->checkForReceive();

    if (received_msg)
    {
        auto t_start = std::chrono::steady_clock::now();
        bool parseSuccess = rob_connection_->parseReceivedMessage();

        if (!parseSuccess)
            return(false);
    }

    return(received_msg);
}

bool KukaKssHardwareInterface::readRSI(const ros::Time time, const ros::Duration period)
{
    bool received_msg = rsi_connection_->checkForReceive();

    if (received_msg)
    {
        auto t_start = std::chrono::steady_clock::now();
        bool parseSuccess = rsi_connection_->parseReceivedMessage();

        if (!parseSuccess)
            return(false);

        if (rt_rsi_pub_->trylock()){
          rt_rsi_pub_->msg_.data = rsi_connection_->getReceivedXML();
          rt_rsi_pub_->unlockAndPublish();
        }

        unsigned int delay = rsi_connection_->getDelay();
        ROS_WARN_COND_NAMED(((delay % 5) == 4), log_name_,"Communication Delay = %d packets", delay);
    }

    return(received_msg);
}



bool KukaKssHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
    //if (rob_connection_->getCommState() != KukaCommHandlerEKI::CycleState::PARSED)
    //    return(false);

    rob_connection_->messagePrepare(period);
    rob_connection_->messageSend();

    if (rt_pub_ipoc_->trylock()){
        rt_pub_ipoc_->msg_.data = KukaRobotStateManager::getRobot(robot_state_id_).getIPOC();
        rt_pub_ipoc_->unlockAndPublish();
    }

#ifdef TEST_COMM_TIMING
    if (rt_eki_pub_ipoc_interval_->trylock()){
        rt_eki_pub_ipoc_interval_->msg_.data = KukaRobotStateManager::getRobot(robot_state_id_).getIpocDuration();
        rt_eki_pub_ipoc_interval_->unlockAndPublish();
    }
#endif

    return true;
}


bool KukaKssHardwareInterface::writeRSI(const ros::Time time, const ros::Duration period)
{
    if (rsi_connection_->getCycleState() != KukaCommHandlerRSI::CycleState::PARSED)
        return(false);

    rsi_connection_->messagePrepare();
    rsi_connection_->messageSend();

    if (rt_pub_ipoc_->trylock()){
        rt_pub_ipoc_->msg_.data = KukaRobotStateManager::getRobot(robot_state_id_).getIPOC();
        rt_pub_ipoc_->unlockAndPublish();
    }

    if (rt_rsi_pub_delay_->trylock()){
        rt_rsi_pub_delay_->msg_.data = rsi_connection_->getDelay();
        rt_rsi_pub_delay_->unlockAndPublish();
    }

#ifdef TEST_COMM_TIMING
    if (rt_rsi_pub_resp_timeP_->trylock()){
        rt_rsi_pub_resp_timeP_->msg_.data = rsi_connection_->getParseTime();
        rt_rsi_pub_resp_timeP_->unlockAndPublish();
    }
    if (rt_rsi_pub_resp_timeU_->trylock()){
        rt_rsi_pub_resp_timeU_->msg_.data = rsi_connection_->getUpdateTime();
        rt_rsi_pub_resp_timeU_->unlockAndPublish();
    }
    if (rt_rsi_pub_resp_timeW_->trylock()){
        rt_rsi_pub_resp_timeW_->msg_.data = rsi_connection_->getSendTime();
        rt_rsi_pub_resp_timeW_->unlockAndPublish();
    }
#endif

  return true;
}


void KukaKssHardwareInterface::start()
{
    ROS_INFO_THROTTLE_NAMED(5.0, log_name_, "Waiting for robot EKI server...");
    bool successConnect = rob_connection_->startComm();
    while (!successConnect)
    {
        ROS_WARN_NAMED(log_name_, "EKI Server Connection failed.  Attempting to reconnect.");
        rob_connection_->shutdownComm();

        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        successConnect = rob_connection_->startComm();
    }
    ROS_INFO_NAMED(log_name_, "EKI server connected. Initializing communication.");

    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Init), 0);
    rob_connection_->messageSend();
    //ROS_INFO_NAMED(log_name_, "Sent initial EKI connection message.");

    //Wait until we get data
    bool received_message = false;
    // Expect 3 initialization messages: Init, Status, State
    // tracking of last message type in kuka_comm_handler_eki.  Change this while loop to validate that all 3 types were received.
    kuka_commands& cmdNames = kuka_commands::getInstance();
    bool gotInit1 = false;
    bool gotInit2 = false;
    bool gotInit3 = false;
    bool gotInit4 = false;
    bool gotStatus = false;
    bool gotState = false;

    int message_count=0;
    while(message_count < 15)
    {
        received_message = rob_connection_->checkForReceive();
        if (received_message)
        {
            ROS_DEBUG_NAMED(log_name_, "start:Received message %d from robot.", message_count);
            received_message = rob_connection_->parseReceivedMessage();
            message_count++;
            gotInit1 = (cmdNames.getTimestamp(kuka_commands::ROSC_SInit) > 0);
            ROS_DEBUG_NAMED(log_name_, "start:ROSC_SInit ts = %lld.", cmdNames.getTimestamp(kuka_commands::ROSC_SInit));
            gotInit2 = (cmdNames.getTimestamp(kuka_commands::ROSC_SInitMod) > 0);
            ROS_DEBUG_NAMED(log_name_, "start:ROSC_SInitMod ts = %lld.", cmdNames.getTimestamp(kuka_commands::ROSC_SInitMod));
            gotInit3 = (cmdNames.getTimestamp(kuka_commands::ROSC_SInitName) > 0);
            ROS_DEBUG_NAMED(log_name_, "start:ROSC_SInitName ts = %lld.", cmdNames.getTimestamp(kuka_commands::ROSC_SInitName));
            gotInit4 = (cmdNames.getTimestamp(kuka_commands::ROSC_SInitRobV) > 0);
            ROS_DEBUG_NAMED(log_name_, "start:ROSC_SInitRobV ts = %lld.", cmdNames.getTimestamp(kuka_commands::ROSC_SInitRobV));
            gotStatus = (cmdNames.getTimestamp(kuka_commands::ROSC_Sstatus) > 0);
            ROS_DEBUG_NAMED(log_name_, "start:ROSC_Sstatus ts = %lld.", cmdNames.getTimestamp(kuka_commands::ROSC_Sstatus));
            gotState = (cmdNames.getTimestamp(kuka_commands::ROSC_SjState) > 0);
            ROS_DEBUG_NAMED(log_name_, "start:ROSC_SjState ts = %lld.", cmdNames.getTimestamp(kuka_commands::ROSC_SjState));
            // When all the necessary init messages are received, exit the loop
            if (gotInit1 && gotInit2 && gotInit3 && gotInit4 && gotStatus && gotState)
                break;
        }
        else
            std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    rob_connection_->checkReceivedVersion();

    // Send an initial response.
    rob_connection_->messagePrepare();
    rob_connection_->messageSend();

    // Set receive timeout to 1 second
    //server_->set_timeout(1000);

    //TODO: Validate robot init parameters against ROS URDF model.

    ROS_INFO_STREAM_NAMED(log_name_, "EKI Successful connection with robot.");
    std::vector<double> init_pos;
    init_pos.resize(12, 0.0);
    {
        std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getRobotMutex());

        // Get the position in Kuka units for display
        for (std::size_t i = 0; i < 12; ++i)
        {
            init_pos[i] = KukaRobotStateManager::getRobot(robot_state_id_).getJointPositionKuka(i);
        }
    }

    ROS_INFO_NAMED(log_name_,
                   "EKI Initial position: A1=%.2f A2=%.2f A3=%.2f A4=%.2f A5=%.2f A6=%.2f E1=%.2f E2=%.2f  E3=%.2f  E4=%.2f  E5=%.2f  E6=%.2f",
                   init_pos[0], init_pos[1], init_pos[2], init_pos[3], init_pos[4], init_pos[5],
                   init_pos[6], init_pos[7], init_pos[8], init_pos[9], init_pos[10],init_pos[11] );

}


void KukaKssHardwareInterface::startRSI()
{
    if (rsi_connection_->getCommState() == KukaCommHandler::INITIALIZED)
    {
        rsi_connection_->startComm();
    }

    //Wait until we get data
    bool received_message = false;
    //while(received_message  == false)
    if (rsi_connection_->getCommState() == KukaCommHandler::LISTENING)
    {
        received_message = rsi_connection_->checkForReceive();
        if (received_message)
            received_message = rsi_connection_->parseReceivedMessage();
        else
            std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    if (received_message)
    {
        rsi_connection_->checkReceivedVersion();

        // Send an initial response.
        rsi_connection_->messagePrepare();
        rsi_connection_->messageSend();

        // Set receive timeout to 1 second
        //TODO: implement timeout in rsi_connection_
        //server_->set_timeout(1000);

        ROS_INFO_STREAM_NAMED(log_name_, "RSI connection established with robot.");
        std::vector<double> init_pos;
        init_pos.resize(12, 0.0);

        // Get the position in Kuka units for display
        for (std::size_t i = 0; i < 12; ++i)
        {
            init_pos[i] = KukaRobotStateManager::getRobot(robot_state_id_).getJointPositionKuka(i);
        }

        ROS_INFO_NAMED(log_name_,
                       "RSI Initial position: A1=%.2f A2=%.2f A3=%.2f A4=%.2f A5=%.2f A6=%.2f E1=%.2f E2=%.2f  E3=%.2f  E4=%.2f  E5=%.2f  E6=%.2f",
                       init_pos[0], init_pos[1], init_pos[2], init_pos[3], init_pos[4], init_pos[5],
                       init_pos[6], init_pos[7], init_pos[8], init_pos[9], init_pos[10],init_pos[11] );

        rsi_did_startup_ = true;
    }
}

void KukaKssHardwareInterface::configure()
{
    bool has_rt = configure_thread_priority();

    std::string serverIP;
    nh_.getParam("kuka/ip_address", serverIP);
    tcp_to_robot_.setupParams("kuka/ip_address", "kuka/server_port", "kuka/server_timeout");
    udp_server_.setupParams("rsi/listen_address", "rsi/listen_port");

    KukaRobotStateManager::getRobot(robot_state_id_).getRobotInfo().controller.model = "KRC";
    KukaRobotStateManager::getRobot(robot_state_id_).getRobotInfo().controller.address = serverIP;
    KukaRobotStateManager::getRobot(robot_state_id_).getRobotInfo().robots[0].address = serverIP;

#ifdef EKI_COMM_XML
    rob_connection_.reset(new KukaCommHandlerEKI(tcp_to_robot_, robot_state_id_, 2, nh_, log_name_));
#elif defined(EKI_COMM_BIN)
    rob_connection_.reset(new KukaCommHandlerEKIBIN(tcp_to_robot_, robot_state_id_, 2, nh_, log_name_));
#endif
    rob_connection_->setup();

    rsi_connection_.reset(new KukaCommHandlerRSI(udp_server_, robot_state_id_, 1, nh_, log_name_));
    rsi_connection_->setup();

    rt_rsi_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc", 3));
    rt_rsi_pub_delay_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int16>(nh_, "rsi_delay", 3));
    rt_pub_ipoc_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int64>(nh_, "ipoc", 3));
    rt_pub_robstatus_.reset(new realtime_tools::RealtimePublisher<industrial_msgs::RobotStatus>(nh_, "robot_status", 3)) ;
    // TODO: Publish control type instead of get_control_type service? - would need change watcher on control type.

#ifdef TEST_COMM_TIMING
    rt_eki_pub_ipoc_interval_.reset(new realtime_tools::RealtimePublisher<std_msgs::Duration>(nh_, "eki_ipoc_interval", 3));
    rt_rsi_pub_resp_timeP_.reset(new realtime_tools::RealtimePublisher<std_msgs::Duration>(nh_, "rsi_response_time_p", 3));
    rt_rsi_pub_resp_timeU_.reset(new realtime_tools::RealtimePublisher<std_msgs::Duration>(nh_, "rsi_response_time_u", 3));
    rt_rsi_pub_resp_timeW_.reset(new realtime_tools::RealtimePublisher<std_msgs::Duration>(nh_, "rsi_response_time_w", 3));
#endif

    // Setup services
    srv_start_motion_ = nh_.advertiseService("start_motion", &KukaKssHardwareInterface::startMotionCB, this);
    srv_stop_motion_ = nh_.advertiseService("stop_motion", &KukaKssHardwareInterface::stopMotionCB, this);
    srv_reset_ = nh_.advertiseService("reset", &KukaKssHardwareInterface::resetCB, this);
    srv_drive_power_on_ = nh_.advertiseService("enable_robot", &KukaKssHardwareInterface::setDrivePowerOnCB, this);
    srv_drive_power_off_ = nh_.advertiseService("disable_robot", &KukaKssHardwareInterface::setDrivePowerOffCB, this);
    srv_get_robot_info_ = nh_.advertiseService("get_robot_info", &KukaKssHardwareInterface::getRobotInfoCB, this);
    // Setup services for switching control mode.
    srv_req_control_type_ = nh_.advertiseService("request_control_type", &KukaKssHardwareInterface::requestControlTypeCB, this);
    srv_get_control_type_ = nh_.advertiseService("get_control_type", &KukaKssHardwareInterface::getControlTypeCB, this);

    //TODO: Setup "shutdown" service to shut down this node.

    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Completed configuration.");
}


bool KukaKssHardwareInterface::startMotionCB(industrial_msgs::StartMotion::Request &request,
                                             industrial_msgs::StartMotion::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: start_motion.");
    std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getStatusMutex());

    KukaRobotState::KukaOpMode opMode = KukaRobotStateManager::getRobot(robot_state_id_).getOpMode();
    if (opMode == KukaRobotState::KukaOpMode::Ext)
    {
        rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Start), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
        rob_connection_->messageSend();
        response.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
    }
    else
    {
        response.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
        ROS_INFO_STREAM_NAMED(log_name_, "start_motion request blocked.  Robot keyswitch must be in EXT mode.");
    }

    return true;
}

bool KukaKssHardwareInterface::stopMotionCB(industrial_msgs::StopMotion::Request &request,
                                            industrial_msgs::StopMotion::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: stop_motion.");
    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Stop), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
    rob_connection_->messageSend();

    // debug: always success.
    response.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

    return true;
}

bool KukaKssHardwareInterface::resetCB(std_srvs::Empty::Request &request,
                                       std_srvs::Empty::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: reset.");
    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Reset), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
    rob_connection_->messageSend();

    return true;
}


bool KukaKssHardwareInterface::setDrivePowerOnCB(industrial_msgs::SetDrivePower::Request &request,
                                                 industrial_msgs::SetDrivePower::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: enable_robot.");
    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_DrivesOn), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
    rob_connection_->messageSend();

    // debug: always success.
    response.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

    return true;
}


bool KukaKssHardwareInterface::setDrivePowerOffCB(industrial_msgs::SetDrivePower::Request &request,
                                                  industrial_msgs::SetDrivePower::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: disable_robot.");
    rob_connection_->messagePrepareCommand(kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_DrivesOff), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
    rob_connection_->messageSend();

    // debug: always success.
    response.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

    return true;
}


bool KukaKssHardwareInterface::getRobotInfoCB(industrial_msgs::GetRobotInfo::Request &request,
                                              industrial_msgs::GetRobotInfo::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: device_info.");

    if (response.robots.capacity() == 0) response.robots.resize(1);
    response = KukaRobotStateManager::getRobot(robot_state_id_).getRobotInfo();

    // debug: always success.
    response.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

    return true;
}


bool KukaKssHardwareInterface::getControlTypeCB(kuka_msgs::GetControlType::Request &request,
                                                kuka_msgs::GetControlType::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: get_control_type.");
    std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getStatusMutex());
    response.control_type = KukaRobotStateManager::getRobot(robot_state_id_).getSelectedControlType();

    return true;
}

bool KukaKssHardwareInterface::requestControlTypeCB(kuka_msgs::RequestControlType::Request &request,
                                                    kuka_msgs::RequestControlType::Response &response)
{
    ROS_INFO_STREAM_NAMED(log_name_, "KukaKssHardwareInterface--Calling service: request_control_type.");
    std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getStatusMutex());

    // Default fail until valid check
    response.code.val = industrial_msgs::ServiceReturnCode::FAILURE;


    // A request to change to current mode is always OK.
    if (KukaRobotStateManager::getRobot(robot_state_id_).getSelectedControlType().control_type == request.request_type.control_type)
    {
        response.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
        // Do nothing: no need to send change request for same mode.
        return true;
    }

    kuka_msgs::ControlState newOpState = determineRobOpState(request.request_type, activeController());
    if (newOpState.control_state == kuka_msgs::ControlState::RosControl_UNKNOWN)
    {
        response.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
        ROS_WARN_STREAM_NAMED(log_name_, "request_control_type failed:  Cant switch to requested mode from current mode.  Try changing ROS Controller.");
        return true;
    }
    KukaRobotStateManager::getRobot(robot_state_id_).setRequestedControlType(request.request_type);
    KukaRobotStateManager::getRobot(robot_state_id_).setRequestedRobOpState(newOpState);
    rob_connection_->messagePrepareOpState(KukaRobotStateManager::getRobot(robot_state_id_).getOpStateStr(newOpState), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
    rob_connection_->messageSend();

    return true;
}




void KukaKssHardwareInterface::debugDumpControllerInfo(const std::list<hardware_interface::ControllerInfo>& ctl_list)
{
    for (auto &list_item : ctl_list)
    {
        ROS_INFO_NAMED(log_name_, "   item: %s, type=%s", list_item.name.c_str(), list_item.type.c_str());
        ROS_INFO_NAMED(log_name_, "      Resources:");
        for (auto &resource:  list_item.claimed_resources)
        {
            ROS_INFO_NAMED(log_name_, "        HW Interface = %s : ", resource.hardware_interface.c_str());
            const std::set<std::string>& iface_resources =  resource.resources;
            for (auto &setData :iface_resources)
            {
                ROS_INFO_NAMED(log_name_, "          %s : ", setData.c_str());
            }
        }
    }

}



bool KukaKssHardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                           const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    ROS_INFO_NAMED(log_name_, "prepareSwitch - start_list");
    debugDumpControllerInfo(start_list);

    ROS_INFO_NAMED(log_name_, "prepareSwitch - stop_list");
    debugDumpControllerInfo(stop_list);

    ROS_INFO_NAMED(log_name_, "active Controller: %s", activeController().c_str());

    std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getStatusMutex());
    // Only a single controller type supported at a time
    // EKI - Only support position_controllers/JointTrajectoryController & pos_vel_controllers/JointTrajectoryController
    // RSI - Support position & velocity controller position_controllers/JointTrajectoryController & velocity_controllers/JointVelocityController
    bool switchOK = true;

    for (auto &start_item : start_list)
    {
        bool checkOK = true;
        switch (KukaRobotStateManager::getRobot(robot_state_id_).getSelectedControlType().control_type)
        {
        case kuka_msgs::ControlType::KUKA_KRL:
            checkOK = (start_item.type == "position_controllers/JointTrajectoryController")
                        || (start_item.type == "pos_vel_controllers/JointTrajectoryController")
                        || (start_item.type == "joint_state_controller/JointStateController");

            break;
        case kuka_msgs::ControlType::KUKA_RSI4ms:
        case kuka_msgs::ControlType::KUKA_RSI12ms:
            checkOK = (start_item.type == "position_controllers/JointTrajectoryController")
                         || (start_item.type == "pos_vel_controllers/JointTrajectoryController")
                         || (start_item.type == "velocity_controllers/JointTrajectoryController")
                         || (start_item.type == "joint_state_controller/JointStateController");
            break;
        default:
            checkOK = false;
            break;
        }
        switchOK = switchOK && checkOK;

        // Dont determineRobOpState if this is the joint_state_controller
        if (start_item.type.find("joint_state_controller")==string::npos)
        {
            kuka_msgs::ControlState newOpState = determineRobOpState(KukaRobotStateManager::getRobot(robot_state_id_).getSelectedControlType(), start_item.type);
            if (newOpState.control_state == kuka_msgs::ControlState::RosControl_UNKNOWN)
            {
                checkOK = false;
                ROS_WARN_STREAM_NAMED(log_name_, "Cant switch to requested mode from current mode. Try changing OpMode with request_control_type service.");
            }
            switchOK = switchOK && checkOK;
        }
    }
    // Handle stopping all controllers
    if (start_list.size() == 0)
    {
        switchOK = true;
    }

    return (switchOK);
}

void KukaKssHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                      const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    ROS_INFO_NAMED(log_name_, "called doSwitch");

    // Manage list of active controllers
    // Remove stopped controllers
    for (auto &controller : stop_list)
    {
        ROS_INFO_NAMED(log_name_, "-->controller_list removing %s, %s", controller.name.c_str(), controller.type.c_str());
        if (controller_list_.erase(controller.name) == 0)
        {
             ROS_WARN_NAMED(log_name_, "Attempted to stop controller '%s' but it was not in the controller list.", controller.name.c_str());
        }
    }
    // Add started controllers
    for (auto &controller : start_list)
    {
        controller_list_[controller.name] = controller.type;
        ROS_INFO_NAMED(log_name_, "-->controller_list added %s, %s", controller.name.c_str(), controller.type.c_str());
    }

    //std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getStatusMutex());

    // Send change request to KRC.
    kuka_msgs::ControlState newOpState = determineRobOpState(KukaRobotStateManager::getRobot(robot_state_id_).getSelectedControlType(), activeController());
    if (newOpState.control_state == kuka_msgs::ControlState::RosControl_UNKNOWN)
    {
        ROS_WARN_STREAM_NAMED(log_name_, "Cant switch to requested mode from current mode.  Try changing OpMode with request_control_type service.");
        return;
    }

    KukaRobotStateManager::getRobot(robot_state_id_).setRequestedRobOpState(newOpState);
    rob_connection_->messagePrepareOpState(KukaRobotStateManager::getRobot(robot_state_id_).getOpStateStr(newOpState), KukaRobotStateManager::getRobot(robot_state_id_).getIPOC());
    rob_connection_->messageSend();
}

hardware_interface::RobotHW::SwitchState KukaKssHardwareInterface::switchResult() const
{
    //ROS_INFO_NAMED(log_name_, "called switchResult()");
    std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(robot_state_id_).getStatusMutex());
    if (KukaRobotStateManager::getRobot(robot_state_id_).getRequestdRobOpState() == KukaRobotStateManager::getRobot(robot_state_id_).getRobOpState())
        return DONE;
    else
        return ONGOING;
}

/** \brief Return (in realtime) the state of the last doSwitch() for a given controller. */
hardware_interface::RobotHW::SwitchState KukaKssHardwareInterface::switchResult(const hardware_interface::ControllerInfo& controller) const
{
    //ROS_INFO_NAMED(log_name_, "called switchResult(controller)");
    return (switchResult());
}


std::string KukaKssHardwareInterface::activeController() const
{
    std::string controller = " ";
    for (auto const &mapitem : controller_list_)
    {
        // Ignore the joint_position reporting controller
        if (mapitem.second == "joint_state_controller/JointStateController")
            continue;
        else
            controller = mapitem.second;
    }
    return(controller);
}

kuka_msgs::ControlState KukaKssHardwareInterface::determineRobOpState(kuka_msgs::ControlType ctrl_type, std::string ROScontroller)
{
    // TODO: Investigate refactoring to send supported control modes and program names from robot side during init ==> This just becomes data driven lookup.
    //  KUKA_KRL supports: position_controllers/JointTrajectoryController, pos_vel_controllers/JointTrajectoryController
    //  KUKA_RSI4ms supports: position_controllers/JointTrajectoryController, pos_vel_controllers/JointTrajectoryController, velocity_controllers/JointTrajectoryController
    //  KUKA_RSI12ms supports: position_controllers/JointTrajectoryController, pos_vel_controllers/JointTrajectoryController, velocity_controllers/JointTrajectoryController
    kuka_msgs::ControlState opMode;
    opMode.control_state = kuka_msgs::ControlState::RosControl_UNKNOWN;

    switch(ctrl_type.control_type)
    {
    case kuka_msgs::ControlType::KUKA_KRL:
        if (ROScontroller == "position_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_KRL_PTP;
        if (ROScontroller == "pos_vel_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_KRL_PTP;
        break;
    case kuka_msgs::ControlType::KUKA_RSI4ms:
        if (ROScontroller == "position_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_RSI4ms_Pos;
        if (ROScontroller == "pos_vel_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_RSI4ms_Pos;
        if (ROScontroller == "velocity_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_RSI4ms_Vel;
        break;
    case kuka_msgs::ControlType::KUKA_RSI12ms:
        if (ROScontroller == "position_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_RSI12ms_Pos;
        if (ROScontroller == "pos_vel_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_RSI12ms_Pos;
        if (ROScontroller == "velocity_controllers/JointTrajectoryController")
            opMode.control_state = kuka_msgs::ControlState::RosControl_RSI12ms_Vel;
        break;
    }
    return(opMode);
}


// Check for real-time kernel & configure thread priority
// returns true if the system has a real-time kernel
bool KukaKssHardwareInterface::configure_thread_priority()
{
    // Check for real-time kernel
    bool has_realtime_kernel = false;
    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    if (realtime_file.is_open())
    {
        realtime_file >> has_realtime_kernel;
    }
    // TODO: create ROS param to specify desired thread priority

    // Get current settings of the thread
    pthread_t self_thread = pthread_self();
    struct sched_param thread_params;
    int policy = 0;
    int rv = pthread_getschedparam(self_thread, &policy, &thread_params);
    if (rv != 0)
        ROS_ERROR_STREAM_NAMED("kuka_hardware_interface", "Unable to get default thread scheduling settings.");
    else
        ROS_INFO_NAMED("kuka_hardware_interface", "Default thread priority = %d.", thread_params.sched_priority);

    if (has_realtime_kernel)
    {
        // Attempt to set the thread as a realtime task
        const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
        if (max_thread_priority == -1)
        {
            ROS_ERROR_STREAM_NAMED("kuka_hardware_interface", "Unable to determine max priority.  No task priority set.");
            return(has_realtime_kernel);
        }
        thread_params.sched_priority = max_thread_priority;
        rv = pthread_setschedparam(self_thread, SCHED_FIFO, &thread_params);
        if (rv != 0)
            ROS_ERROR_STREAM_NAMED("kuka_hardware_interface", "Unable to set task to realtime priority.");

        //Readback check of the thread priority
        policy = 0;
        rv = pthread_getschedparam(self_thread, &policy, &thread_params);
        if (rv != 0)
            ROS_ERROR_STREAM_NAMED("kuka_hardware_interface", "Unable to read back new thread scheduling settings.");

        ROS_INFO_NAMED("kuka_hardware_interface", "New thread priority = %d, policy = %s", thread_params.sched_priority,
                       ((policy == SCHED_FIFO)  ? "SCHED_FIFO" :
                        (policy == SCHED_RR)    ? "SCHED_RR" :
                        (policy == SCHED_OTHER) ? "SCHED_OTHER" :
                        "???")
                       );
    }

    return(has_realtime_kernel);
}


void KukaKssHardwareInterface::testTCPcomm()
{
    // ===================== TEST CODE - TCP Client
    ROS_INFO_STREAM_NAMED("test_tcp_client", "Testing EKI client class.");
    CommLink_TCPClient tcpClient(nh_, "test_tcp_client");

    tcpClient.setupParams("eki/robot_address", "eki/robot_port", "eki/socket_timeout");
    tcpClient.setup();
    ROS_INFO_STREAM_NAMED("test_tcp_client", "Starting EKI client connection.");
    tcpClient.start();

    ROS_INFO_STREAM_NAMED("test_tcp_client", "Sending initial message.");
    std::string sendMsg = "InitialConnect";
    tcpClient.send(sendMsg);

    std::string recvMsg;
    recvMsg.reserve(2048);

    ROS_INFO_STREAM_NAMED("test_tcp_client", "Listening for initial response.");
    auto t_start = std::chrono::steady_clock::now();
    ros::Duration secTimeout;
    secTimeout.fromSec(20.0);
    ros::Duration secElapsed;
    secElapsed.fromSec(0.0);
    int loopCnt = 0;
    size_t bytesRecv = 0;
    bool doLoop = true;
    while(doLoop)
    {
      bytesRecv = tcpClient.receive(recvMsg);
      if (bytesRecv > 0)
      {
          ROS_INFO_NAMED("test_tcp_client", "received message: %s", recvMsg.c_str());
      }
      if (recvMsg.find("go", 0, 1) != std::string::npos )
      {
          doLoop = false;
      }

      auto t_check = std::chrono::steady_clock::now();
      secElapsed.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_check - t_start).count());
      if (secElapsed > secTimeout)
      {
          ROS_INFO_STREAM_NAMED("test_tcp_client", "timeout on TCP test.");
          doLoop = false;
      }
      loopCnt++;
      ROS_INFO_THROTTLE_NAMED(0.5, "test_tcp_client", "Listen Loop %d.", loopCnt);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO_STREAM_NAMED("test_tcp_client", "TCP client test done.");
}

void KukaKssHardwareInterface::testUDPcomm()
{
    // ===================== TEST CODE - UDP Server
    ROS_INFO_STREAM_NAMED("test_udp_server", "Testing UDP Server class.");
    CommLink_UDPServer udpServer(nh_, "test_udp_server");

    udpServer.setupParams("test_udp/listen_address", "test_udp/listen_port");
    udpServer.setup();
    ROS_INFO_STREAM_NAMED("test_udp_server", "Starting UDP Server connection.");
    udpServer.start();

    ROS_INFO_STREAM_NAMED("test_udp_server", "Waiting for initial message.");

    std::string recvMsg;
    recvMsg.reserve(2048);

    auto t_start = std::chrono::steady_clock::now();
    ros::Duration secTimeout;
    secTimeout.fromSec(20.0);
    ros::Duration secElapsed;
    secElapsed.fromSec(0.0);
    int loopCnt = 0;
    size_t bytesRecv = 0;
    bool doLoop = true;
    while(doLoop)
    {
      bytesRecv = udpServer.receive(recvMsg);
      if (bytesRecv > 0)
      {
          ROS_INFO_NAMED("test_udp_server", "received message: %s", recvMsg.c_str());

          std::string sendMsg = "Acknowledge response: " + recvMsg;
          udpServer.send(sendMsg);
      }
      if (recvMsg.find("go", 0, 1) != std::string::npos )
      {
          doLoop = false;
      }

      auto t_check = std::chrono::steady_clock::now();
      secElapsed.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_check - t_start).count());
      if (secElapsed > secTimeout)
      {
          ROS_INFO_STREAM_NAMED("test_udp_server", "timeout on UDP test.");
          doLoop = false;
      }
      loopCnt++;
      ROS_INFO_THROTTLE_NAMED(0.5, "test_udp_server", "Listen Loop %d.", loopCnt);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO_STREAM_NAMED("test_udp_server", "UDP Server test done.");
}


} // namespace kuka_kss_hw_interface
