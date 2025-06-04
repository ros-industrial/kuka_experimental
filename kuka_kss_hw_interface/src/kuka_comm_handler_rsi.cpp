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



#include <kuka_kss_hw_interface/kuka_comm_handler_rsi.h>

namespace kuka_kss_hw_interface
{



KukaCommHandlerRSI::KukaCommHandlerRSI(CommunicationLink& comm, int id, int priority, const ros::NodeHandle& nh, std::string log_id):
    KukaCommHandler(comm, id, priority, nh, log_id),
    // Note: actual version is set below with setVersionInfo() in the constructor...
    RSI_INTERFACE_VERSION_("V0000ROS"), version_num_str_("0000"), version_num_str_robot_("0"), delay(0), did_init(false), cycleState_(UNKNOWN), did_receive_msg_(false),
    initial_positions_(12,0.0), cmd_corrections_(12,0.0), cmd_corrections_str_(12),
    cart_position_(6,0.0), initial_cart_position_(6,0.0)
{
    ROS_INFO_STREAM_NAMED(logging_name_, "Installing RSI communication link.");
    t_received_ = std::chrono::steady_clock::now();
    t_received_last_ = t_received_;
    t_parseComplete_ = t_received_;
    t_startPrepare_ = t_received_;
    t_sent_ = t_received_;
    responseXML_.reset(new XmlTemplateXpath());
    recv_msg_.clear();
    recv_xml_.clear();

    // Default end of receive message.
    startStr_ = "<Rob Type";
    endStr_ = "</Rob>";

    try {
        // Default XML response telegram
        std::string xmlStr;
        xmlStr = R"(<Sen Type="V2001ROS"><AK A1="-100.495300" A2="-100.244600" A3="-165.559500" A4="-112.204200" A5="-195.155600" A6="-113.062300" E1="1000.000000" E2="1000.000000" E3="1000.000000" E4="1000.000000" E5="1000.000000" E6="1000.000000" /><STAT VER="2001" STOP="0"/><IPOC>26147434</IPOC></Sen>)";
        setXmlResponseTemplate(xmlStr);

        // Must be of the form VnnnnROS
        setVersionInfo("V2004ROS");
    } catch (std::exception& ex){
        ROS_ERROR_STREAM_NAMED(logging_name_, "Error setting response XML.");
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
    }
    state_ = DISCONNECTED;
}


KukaCommHandlerRSI::~KukaCommHandlerRSI()
{

}

void KukaCommHandlerRSI::setVersionInfo(std::string ver)
{
    RSI_INTERFACE_VERSION_ = ver;
    // Extract the version number from format VnnnnROS
    version_num_str_ = RSI_INTERFACE_VERSION_.substr(1,4);

    responseXML_->set("/Sen@Type", (char*)RSI_INTERFACE_VERSION_.c_str());
    responseXML_->set("/Sen/STAT@VER", (char *)version_num_str_.c_str());
    ROS_INFO_NAMED(logging_name_,"Kuka ROS RSI interface version %s.", RSI_INTERFACE_VERSION_.c_str());
}




bool KukaCommHandlerRSI::checkReceivedVersion()
{
    bool versionMatch = (version_num_str_ == version_num_str_robot_);
    if (!versionMatch)
    {
        ROS_WARN_NAMED(logging_name_, "The RSI interface version on the robot is '%s' but version '%s' is expected by this node. "
                  "Make sure correct version is loaded in KRL and in robot C:/KRC/Roboter/Config/User/Common/SensorInterface",
                  version_num_str_robot_.c_str(), version_num_str_.c_str());
    }
    return(versionMatch);
}


bool KukaCommHandlerRSI::setup()
{
    bool setupOK;
    setupOK = comm_link_.setup();
    endStr_len_ = endStr_.length();
    if (setupOK)
    {
        state_ = INITIALIZED;
        cycleState_ = CycleState::INIT;
    }
    return(setupOK);
}

bool KukaCommHandlerRSI::startComm()
{
    bool started;
    started = comm_link_.start();
    if (started)
    {
        state_ = LISTENING;
        cycleState_ = CycleState::WAITING;
    }
    ROS_INFO_STREAM_NAMED(logging_name_, "Started Kuka RSI communication link.");
    return(started);
}


bool KukaCommHandlerRSI::shutdownComm()
{
    bool success = comm_link_.stop();

    state_ = DISCONNECTED;
    cycleState_ = CycleState::INIT;
    did_init = false;
    ROS_INFO_STREAM_NAMED(logging_name_, "Stopped Kuka RSI communication link.");
    return(success);
}

bool KukaCommHandlerRSI::checkForReceive()
{
    size_t total_bytes_recv=0;
    size_t bytes_recv=0;
    bool checkForMore = true;
    // Get all buffered data. (In case delays happened there could be multiple packets)
    while (checkForMore)
    {
        recv_comm_buff_.clear();
        bytes_recv = comm_link_.receive(recv_comm_buff_);
        if (bytes_recv <= 0)
            checkForMore = false;
        else
        {
            total_bytes_recv += bytes_recv;
            recv_msg_.append(recv_comm_buff_);
        }
    }

    if (total_bytes_recv <= 0)
    {
        // Timeout check & reset for no received data
        ros::Duration dur;
        if (getResponseTime() > dur.fromSec(0.200))
        {
            state_ = LISTENING;
        }
        return(false);
    }

    // Connectionless UDP - Receipt of some data indicates we are connected
    state_ = CONNECTED;
    cycleState_ = CycleState::RECEIVING;

    // Handling for partial or multiple messages in buffer.
    // LIFO - Only read last message.  Others will be considered late/lost.
    size_t startStrPos;
    size_t endStrPos;
    // A quick search for the XML closing
    endStrPos = recv_msg_.rfind(endStr_);
    // not found
    if (endStrPos == std::string::npos)
    {
        //ROS_DEBUG_NAMED(logging_name_, "Partial packet: %s ", recv_msg_.c_str());
        did_receive_msg_ = false;
        return(did_receive_msg_);
    }
    startStrPos =  recv_msg_.rfind(startStr_, endStrPos);
    // not found
    if (startStrPos == std::string::npos)
    {
        std::string msg = "RSI Incomplete XML packet. Start not found.";
        ROS_ERROR_STREAM_NAMED(logging_name_, msg);
        throw std::runtime_error(msg);
    }

    cycleState_ = CycleState::RECEIVED;
    t_received_last_ = t_received_;
    t_received_ = std::chrono::steady_clock::now();
    did_receive_msg_ = true;
    recv_xml_.clear();
    recv_xml_ = recv_msg_.substr(startStrPos, endStrPos + endStr_len_);
    // Remove the xml packet from the receive_buffer - handles cases where multiple messages arrive together
    recv_msg_.erase(0, endStrPos + endStr_len_);
    //ROS_DEBUG_NAMED(logging_name_, "Ready to process: %s ", recv_xml_.c_str());

    return(did_receive_msg_);
}

bool KukaCommHandlerRSI::wasReceived()
{
    return(did_receive_msg_);
}

bool KukaCommHandlerRSI::parseReceivedMessage()
{
    if (!did_receive_msg_)
    {
        //ROS_DEBUG_STREAM_NAMED(logging_name_, "Attempt to parse before received message.");
        return(false);
    }

    XmlTemplateXpath parsedXML;
    try {
        parsedXML.setXmlTemplate(recv_xml_.c_str());
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_NAMED(logging_name_, "RSI XML Parse Error: %s", ex.what());
        //throw(ex);
        return(false);
    }

    try {
        std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

        // Extract axis specific actual position =========================================
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(0, std::stod(parsedXML.get("/Rob/AIPos@A1")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(1, std::stod(parsedXML.get("/Rob/AIPos@A2")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(2, std::stod(parsedXML.get("/Rob/AIPos@A3")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(3, std::stod(parsedXML.get("/Rob/AIPos@A4")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(4, std::stod(parsedXML.get("/Rob/AIPos@A5")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(5, std::stod(parsedXML.get("/Rob/AIPos@A6")));
        // External axes
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(6 , std::stod(parsedXML.get("/Rob/EIPos@E1")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(7 , std::stod(parsedXML.get("/Rob/EIPos@E2")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(8 , std::stod(parsedXML.get("/Rob/EIPos@E3")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(9 , std::stod(parsedXML.get("/Rob/EIPos@E4")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(10, std::stod(parsedXML.get("/Rob/EIPos@E5")));
        KukaRobotStateManager::getRobot(rob_id_).setJointPosition(11, std::stod(parsedXML.get("/Rob/EIPos@E6")));

        // Extract axis specific velocity =========================================
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(0, std::stod(parsedXML.get("/Rob/AIVel@A1")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(1, std::stod(parsedXML.get("/Rob/AIVel@A2")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(2, std::stod(parsedXML.get("/Rob/AIVel@A3")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(3, std::stod(parsedXML.get("/Rob/AIVel@A4")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(4, std::stod(parsedXML.get("/Rob/AIVel@A5")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(5, std::stod(parsedXML.get("/Rob/AIVel@A6")));
        // External axes
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(6 , std::stod(parsedXML.get("/Rob/AIVel@E1")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(7 , std::stod(parsedXML.get("/Rob/AIVel@E2")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(8 , std::stod(parsedXML.get("/Rob/AIVel@E3")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(9 , std::stod(parsedXML.get("/Rob/AIVel@E4")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(10, std::stod(parsedXML.get("/Rob/AIVel@E5")));
        KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(11, std::stod(parsedXML.get("/Rob/AIVel@E6")));

        // Extract axis specific effort =========================================
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(0, std::stod(parsedXML.get("/Rob/AIEff@A1")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(1, std::stod(parsedXML.get("/Rob/AIEff@A2")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(2, std::stod(parsedXML.get("/Rob/AIEff@A3")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(3, std::stod(parsedXML.get("/Rob/AIEff@A4")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(4, std::stod(parsedXML.get("/Rob/AIEff@A5")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(5, std::stod(parsedXML.get("/Rob/AIEff@A6")));
        // External axes
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(6 ,std::stod(parsedXML.get("/Rob/AIEff@E1")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(7 ,std::stod(parsedXML.get("/Rob/AIEff@E2")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(8 ,std::stod(parsedXML.get("/Rob/AIEff@E3")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(9 ,std::stod(parsedXML.get("/Rob/AIEff@E4")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(10,std::stod(parsedXML.get("/Rob/AIEff@E5")));
        KukaRobotStateManager::getRobot(rob_id_).setJointEffort(11,std::stod(parsedXML.get("/Rob/AIEff@E6")));

        if (!did_init)
        {
            // Store robot side version number.
            version_num_str_robot_ = parsedXML.get("/Rob/STAT@VER");

            // Extract axis specific setpoint position ========================================
            initial_positions_[0] = std::stod(parsedXML.get("/Rob/ASPos@A1"));
            initial_positions_[1] = std::stod(parsedXML.get("/Rob/ASPos@A2"));
            initial_positions_[2] = std::stod(parsedXML.get("/Rob/ASPos@A3"));
            initial_positions_[3] = std::stod(parsedXML.get("/Rob/ASPos@A4"));
            initial_positions_[4] = std::stod(parsedXML.get("/Rob/ASPos@A5"));
            initial_positions_[5] = std::stod(parsedXML.get("/Rob/ASPos@A6"));
            // External axes
            initial_positions_[6]  = std::stod(parsedXML.get("/Rob/ESPos@E1"));
            initial_positions_[7]  = std::stod(parsedXML.get("/Rob/ESPos@E2"));
            initial_positions_[8]  = std::stod(parsedXML.get("/Rob/ESPos@E3"));
            initial_positions_[9]  = std::stod(parsedXML.get("/Rob/ESPos@E4"));
            initial_positions_[10] = std::stod(parsedXML.get("/Rob/ESPos@E5"));
            initial_positions_[11] = std::stod(parsedXML.get("/Rob/ESPos@E6"));

            // Extract cartesian planned position ==============================================
            initial_cart_position_[0] = std::stod(parsedXML.get("/Rob/RSol@X"));
            initial_cart_position_[1] = std::stod(parsedXML.get("/Rob/RSol@Y"));
            initial_cart_position_[2] = std::stod(parsedXML.get("/Rob/RSol@Z"));
            initial_cart_position_[3] = std::stod(parsedXML.get("/Rob/RSol@A"));
            initial_cart_position_[4] = std::stod(parsedXML.get("/Rob/RSol@B"));
            initial_cart_position_[5] = std::stod(parsedXML.get("/Rob/RSol@C"));

            // Initialize the command positions with the actual position read above
            std::vector<double> jpos = KukaRobotStateManager::getRobot(rob_id_).getJointPositions();
            for (std::size_t i = 0; i < 12; ++i)
            {
                *(KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionPtr(i)) = jpos[i];
            }

            did_init = true;

        }
        // Extract cartesian actual position ===============================================
        cart_position_[0] = std::stod(parsedXML.get("/Rob/RIst@X"));
        cart_position_[1] = std::stod(parsedXML.get("/Rob/RIst@Y"));
        cart_position_[2] = std::stod(parsedXML.get("/Rob/RIst@Z"));
        cart_position_[3] = std::stod(parsedXML.get("/Rob/RIst@A"));
        cart_position_[4] = std::stod(parsedXML.get("/Rob/RIst@B"));
        cart_position_[5] = std::stod(parsedXML.get("/Rob/RIst@C"));

        // Get the communication delay metric
        delay = std::stoi(parsedXML.get("/Rob/Delay@D"));


        // Get the IPOC timestamp
        KukaRobotStateManager::getRobot(rob_id_).setIPOC(std::stoull(parsedXML.get("/Rob/IPOC")));

        t_parseComplete_ = std::chrono::steady_clock::now();
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_NAMED(logging_name_, "RSI XML problem extracting data: %s", ex.what());
        throw(ex);
        return(false);
    }

    cycleState_ = CycleState::PARSED;
    // reset flag after data was parsed
    did_receive_msg_ = false;
    return(true);
}


void KukaCommHandlerRSI::setXmlResponseTemplate(std::string xmlStr)
{
  response_XML_template_ = xmlStr;
  responseXML_->setXmlTemplate(response_XML_template_);
}

std::string KukaCommHandlerRSI::getXmlResponseTemplate()
{
  return(response_XML_template_);
}


void KukaCommHandlerRSI::calcCommandCorrectionsPos()
{
    for (std::size_t j = 0; j < 12; ++j)
    {
      cmd_corrections_[j] = KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(j) - initial_positions_[j];
    }
}


bool KukaCommHandlerRSI::messagePrepare()
{
    t_startPrepare_ = std::chrono::steady_clock::now();

    try{
        std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

        //TODO: add calcs for other control methods
        // For position control
        calcCommandCorrectionsPos();

        cmd_corrections_str_[0] = std::to_string( cmd_corrections_[0]);
        cmd_corrections_str_[1] = std::to_string( cmd_corrections_[1]);
        cmd_corrections_str_[2] = std::to_string( cmd_corrections_[2]);
        cmd_corrections_str_[3] = std::to_string( cmd_corrections_[3]);
        cmd_corrections_str_[4] = std::to_string( cmd_corrections_[4]);
        cmd_corrections_str_[5] = std::to_string( cmd_corrections_[5]);
        cmd_corrections_str_[6] = std::to_string( cmd_corrections_[6]);
        cmd_corrections_str_[7] = std::to_string( cmd_corrections_[7]);
        cmd_corrections_str_[8] = std::to_string( cmd_corrections_[8]);
        cmd_corrections_str_[9] = std::to_string( cmd_corrections_[9]);
        cmd_corrections_str_[10] = std::to_string(cmd_corrections_[10]);
        cmd_corrections_str_[11] = std::to_string(cmd_corrections_[11]);
        ipoc_str_ = std::to_string(KukaRobotStateManager::getRobot(rob_id_).getIPOC());

        if (KukaRobotStateManager::getRobot(rob_id_).getRequestdRobOpState() != KukaRobotStateManager::getRobot(rob_id_).getRobOpState())
            stop_str_="1";
        else
            stop_str_="0";

        responseXML_->set("/Sen/AK@A1", (char *)cmd_corrections_str_[0].c_str());
        responseXML_->set("/Sen/AK@A2", (char *)cmd_corrections_str_[1].c_str());
        responseXML_->set("/Sen/AK@A3", (char *)cmd_corrections_str_[2].c_str());
        responseXML_->set("/Sen/AK@A4", (char *)cmd_corrections_str_[3].c_str());
        responseXML_->set("/Sen/AK@A5", (char *)cmd_corrections_str_[4].c_str());
        responseXML_->set("/Sen/AK@A6", (char *)cmd_corrections_str_[5].c_str());
        responseXML_->set("/Sen/AK@E1", (char *)cmd_corrections_str_[6].c_str());
        responseXML_->set("/Sen/AK@E2", (char *)cmd_corrections_str_[7].c_str());
        responseXML_->set("/Sen/AK@E3", (char *)cmd_corrections_str_[8].c_str());
        responseXML_->set("/Sen/AK@E4", (char *)cmd_corrections_str_[9].c_str());
        responseXML_->set("/Sen/AK@E5", (char *)cmd_corrections_str_[10].c_str());
        responseXML_->set("/Sen/AK@E6", (char *)cmd_corrections_str_[11].c_str());

        // Handle graceful exit from RSI
        responseXML_->set("/Sen/STAT@STOP", (char *)stop_str_.c_str());

        responseXML_->set("/Sen/IPOC/", (char *)ipoc_str_.c_str());
        //ROS_INFO_NAMED( logging_name_, "RSI response IPOC=%s", ipoc_str_.c_str());

        send_comm_buff_ = responseXML_->asXml();
        //ROS_INFO_NAMED( logging_name_, "send_comm_buff_=%s", send_comm_buff_.c_str());
        cycleState_ = CycleState::PREPARED;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
        return(false);
    }
    return(true);
}

bool KukaCommHandlerRSI::messageSend()
{
    bool sendOK;
    size_t bytesSent;
    bytesSent = comm_link_.send(send_comm_buff_);
    sendOK = (bytesSent>0);
    t_sent_ = std::chrono::steady_clock::now();
    cycleState_ = CycleState::SENT;
    return(sendOK);
}


ros::Duration KukaCommHandlerRSI::getCommCycle()
{
    ros::Duration dur;
    dur.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_received_ - t_received_last_).count());
    return(dur);
}

ros::Duration KukaCommHandlerRSI::getParseTime()
{
    ros::Duration dur;
    dur.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_parseComplete_ - t_received_).count());
    return(dur);
}

ros::Duration KukaCommHandlerRSI::getUpdateTime()
{
    ros::Duration dur;
    dur.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_startPrepare_ - t_parseComplete_).count());
    return(dur);
}

ros::Duration KukaCommHandlerRSI::getSendTime()
{
    ros::Duration dur;
    dur.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_sent_ - t_startPrepare_).count());
    return(dur);
}

ros::Duration KukaCommHandlerRSI::getResponseTime()
{
    ros::Duration dur;
    dur.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_sent_ - t_received_).count());
    return(dur);
}


} //namespace kuka_kss_hw_interface

