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



#include <kuka_kss_hw_interface/kuka_commands.h>
#include <kuka_kss_hw_interface/kuka_comm_handler_eki.h>

namespace kuka_kss_hw_interface
{



KukaCommHandlerEKI::KukaCommHandlerEKI(CommunicationLink& comm, int id, int priority, const ros::NodeHandle& nh, std::string log_id):
    KukaCommHandler(comm, id, priority, nh, log_id),
    // Note: actual version is set below with setVersionInfo() in the constructor...
    EKI_INTERFACE_VERSION_("V0000ROS"), version_num_str_("2002"), version_num_str_robot_("0"), did_init(false), cycleState_(UNKNOWN), did_receive_msg_(false),
    cmd_corrections_str_(12), duration_(0.200)
{
    ROS_INFO_STREAM_NAMED(logging_name_, "Starting Kuka communication link.");
    t_received_ = std::chrono::steady_clock::now();
    t_received_last_ = t_received_;
    responseXML_.reset(new XmlTemplateXpath());
    commandXML_.reset(new XmlTemplateXpath());
    opStateXML_.reset(new XmlTemplateXpath());
    recv_msg_.clear();
    recv_xml_.clear();

    // Default end of receive message.
    startStr_ = "<Robot>";
    endStr_ = "</Robot>";

    try {
        // Default XML response telegram
        std::string xmlStr;
        xmlStr = R"(<ROS REQTYPE="CMD-----" ID="123456789012345"><Command dt="0.200"><Pos A1="-100.495300" A2="-100.244600" A3="-165.559500" A4="-112.204200" A5="-195.155600" A6="-113.062300" E1="1000.000000" E2="1000.000000" E3="1000.000000" E4="1000.000000" E5="1000.000000" E6="1000.000000"/></Command></ROS>)";
        setXmlResponseTemplate(xmlStr);

        // TODO - Consider separating command & opstate to separte class as a ROS service
        std::string commandStr;
        commandStr = R"(<ROS REQTYPE="COM-----" ID="123456789012345">NA</ROS>)";
        setXmlCommandTemplate(commandStr);

        std::string opStateStr;
        opStateStr = R"(<ROS REQTYPE="CHANGE--" ID="123456789012345"><OpState Type="RSI4ms_Pos">NA</OpState></ROS>)";
        setXmlOpStateTemplate(opStateStr);

        // Must be of the form VnnnnROS
        setVersionInfo("V2004ROS");
    } catch (std::exception& ex){
        ROS_ERROR_STREAM_NAMED(logging_name_, "Error setting response XML.");
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
    }
    //ROS_INFO_STREAM_NAMED(logging_name_, "Completed KukaCommHandlerEKI constructor.");
}


KukaCommHandlerEKI::~KukaCommHandlerEKI()
{

}

void KukaCommHandlerEKI::setVersionInfo(std::string ver)
{
    EKI_INTERFACE_VERSION_ = ver;
    // Extract the version number from format VnnnnROS
    version_num_str_ = EKI_INTERFACE_VERSION_.substr(1,4);

    ROS_INFO_NAMED(logging_name_,"Kuka ROS EKI XML interface version %s.", EKI_INTERFACE_VERSION_.c_str());
}




bool KukaCommHandlerEKI::checkReceivedVersion()
{
    bool versionMatch = (version_num_str_ == version_num_str_robot_);
    if (!versionMatch)
    {
        ROS_WARN_NAMED(logging_name_, "The EKI interface version on the robot is '%s' but version '%s' is expected by this node. "
                  "Make sure correct version is loaded in KRL and in robot C:/KRC/Roboter/Config/User/Common/EthernetKRL",
                  version_num_str_robot_.c_str(), version_num_str_.c_str());
    }
    return(versionMatch);
}


bool KukaCommHandlerEKI::setup()
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

bool KukaCommHandlerEKI::startComm()
{
    bool started;
    started = comm_link_.start();
    if (started)
    {
        state_ = LISTENING;
        cycleState_ = CycleState::WAITING;
        ROS_INFO_STREAM_NAMED(logging_name_, "Started Kuka EKI communication link.");
    }
    else
    {
        ROS_WARN_STREAM_NAMED(logging_name_, "Kuka EKI startComm connection unsuccessful.");
    }
    return(started);
}


bool KukaCommHandlerEKI::shutdownComm()
{
    bool success = comm_link_.stop();

    state_ = LISTENING;
    cycleState_ = CycleState::INIT;
    did_init = false;
    ROS_INFO_STREAM_NAMED(logging_name_, "Stopped Kuka RSI communication link.");
    return(success);
}

bool KukaCommHandlerEKI::checkForReceive()
{

    size_t total_bytes_recv=0;
    size_t bytes_recv=0;
    bool checkForMore = true;
    // Get all buffered data. (In case delays happened there could be multiple packets)
    while (checkForMore)
    {
        recv_comm_buff_.clear();
        bytes_recv = comm_link_.receive(recv_comm_buff_);
        if (bytes_recv > 0)
        {
            total_bytes_recv += bytes_recv;
            recv_msg_.append(recv_comm_buff_);
        }
        else
        {
            checkForMore = false;
        }
    }

    if (total_bytes_recv <= 0)
    {
        // Timeout check & reset for no received data
        if (!comm_link_.is_connected())
            state_ = DISCONNECTED;
        return(false);
    }

    // Receipt of some data indicates we are connected
    state_ = CONNECTED;
    cycleState_ = CycleState::RECEIVING;

    // Handling for partial or multiple messages in buffer.
    // LIFO - Only read last message.  Others will be considered late/lost.
    size_t startStrPos;
    size_t endStrPos;
    // A quick search for the XML closing
    endStrPos = recv_msg_.find(endStr_);
    // not found
    if (endStrPos == std::string::npos)
    {
        //ROS_DEBUG_NAMED(logging_name_, "Partial packet: %s ", recv_msg_.c_str());
        did_receive_msg_ = false;
        return(did_receive_msg_);
    }
    startStrPos =  recv_msg_.find(startStr_);
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
    ROS_DEBUG_NAMED(logging_name_, "Ready to process: %s ", recv_xml_.c_str());

    return(did_receive_msg_);
}

bool KukaCommHandlerEKI::wasReceived()
{
    return(did_receive_msg_);
}

bool KukaCommHandlerEKI::parseReceivedMessage()
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
        ROS_ERROR_NAMED(logging_name_, "EKI XML Parse Error: %s", ex.what());
        throw(ex);
        return(false);
    }

    // Get type of message based on inner XML tag
    std::string messageType = "unknown";
    try {
        xml_node<> *robNode = parsedXML.getNode("/Robot");
        xml_node<> *childNode = robNode->first_node();
        messageType = childNode->name();
        ROS_DEBUG_NAMED(logging_name_, "Recognized message type: %s", messageType.c_str());
    } catch (std::exception& ex) {
            ROS_ERROR_NAMED(logging_name_, "EKI XML problem extracting message type: %s", ex.what());
            throw(ex);
            return(false);
    }


    try {
        bool messageHandled=false;
        //TODO: Change XML structure to use common command names between XML tags & binary.
        kuka_commands& cmdNames = kuka_commands::getInstance();


        // Handle the robot position state update
        if (messageType == "State")
        {
            messageHandled=true;
            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

            // Get the IPOC timestamp
            unsigned long long cur_ipoc = std::stoull(parsedXML.get("/Robot/State@IPOC"));
            ROS_DEBUG_NAMED(logging_name_, "State msg at IPOC= %llu", cur_ipoc);
            KukaRobotStateManager::getRobot(rob_id_).setIPOC(cur_ipoc);

            // Extract axis specific actual position =========================================
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(0, stod_safe(parsedXML.get("/Robot/State/Pos@A1")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(1, stod_safe(parsedXML.get("/Robot/State/Pos@A2")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(2, stod_safe(parsedXML.get("/Robot/State/Pos@A3")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(3, stod_safe(parsedXML.get("/Robot/State/Pos@A4")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(4, stod_safe(parsedXML.get("/Robot/State/Pos@A5")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(5, stod_safe(parsedXML.get("/Robot/State/Pos@A6")));
            // External axes
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(6 , stod_safe(parsedXML.get("/Robot/State/Pos@E1")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(7 , stod_safe(parsedXML.get("/Robot/State/Pos@E2")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(8 , stod_safe(parsedXML.get("/Robot/State/Pos@E3")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(9 , stod_safe(parsedXML.get("/Robot/State/Pos@E4")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(10, stod_safe(parsedXML.get("/Robot/State/Pos@E5")));
            KukaRobotStateManager::getRobot(rob_id_).setJointPosition(11, stod_safe(parsedXML.get("/Robot/State/Pos@E6")));

            // Extract axis specific velocity =========================================
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(0, stod_safe(parsedXML.get("/Robot/State/Vel@A1")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(1, stod_safe(parsedXML.get("/Robot/State/Vel@A2")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(2, stod_safe(parsedXML.get("/Robot/State/Vel@A3")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(3, stod_safe(parsedXML.get("/Robot/State/Vel@A4")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(4, stod_safe(parsedXML.get("/Robot/State/Vel@A5")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(5, stod_safe(parsedXML.get("/Robot/State/Vel@A6")));
            // External axes
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(6 , stod_safe(parsedXML.get("/Robot/State/Vel@E1")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(7 , stod_safe(parsedXML.get("/Robot/State/Vel@E2")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(8 , stod_safe(parsedXML.get("/Robot/State/Vel@E3")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(9 , stod_safe(parsedXML.get("/Robot/State/Vel@E4")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(10, stod_safe(parsedXML.get("/Robot/State/Vel@E5")));
            KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(11, stod_safe(parsedXML.get("/Robot/State/Vel@E6")));

            // Extract axis specific effort =========================================
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(0, stod_safe(parsedXML.get("/Robot/State/Eff@A1")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(1, stod_safe(parsedXML.get("/Robot/State/Eff@A2")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(2, stod_safe(parsedXML.get("/Robot/State/Eff@A3")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(3, stod_safe(parsedXML.get("/Robot/State/Eff@A4")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(4, stod_safe(parsedXML.get("/Robot/State/Eff@A5")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(5, stod_safe(parsedXML.get("/Robot/State/Eff@A6")));
            // External axes
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(6 , stod_safe(parsedXML.get("/Robot/State/Eff@E1")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(7 , stod_safe(parsedXML.get("/Robot/State/Eff@E2")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(8 , stod_safe(parsedXML.get("/Robot/State/Eff@E3")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(9 , stod_safe(parsedXML.get("/Robot/State/Eff@E4")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(10, stod_safe(parsedXML.get("/Robot/State/Eff@E5")));
            KukaRobotStateManager::getRobot(rob_id_).setJointEffort(11, stod_safe(parsedXML.get("/Robot/State/Eff@E6")));

            // Extract cartesian actual position ===============================================
            //cart_position_[0] = stod_safe(parsedXML.get("/Robot/State/RIst@X"));
            //cart_position_[1] = stod_safe(parsedXML.get("/Robot/State/RIst@Y"));
            //cart_position_[2] = stod_safe(parsedXML.get("/Robot/State/RIst@Z"));
            //cart_position_[3] = stod_safe(parsedXML.get("/Robot/State/RIst@A"));
            //cart_position_[4] = stod_safe(parsedXML.get("/Robot/State/RIst@B"));
            //cart_position_[5] = stod_safe(parsedXML.get("/Robot/State/RIst@C"));

            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SjState, cur_ipoc);

            if (!did_init)
            {
                // Initialize the command positions with the actual position read above
                std::vector<double> jpos = KukaRobotStateManager::getRobot(rob_id_).getJointPositions();
                for (std::size_t i = 0; i < 12; ++i)
                {
                    *(KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionPtr(i)) = jpos[i];
                }

                did_init = true;
            }
        }

        // Handle robot status changes
        if (messageType == "Status")
        {
            messageHandled=true;

            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getStatusMutex());

            ROS_INFO_NAMED(logging_name_, "Received Status Message: %s", recv_xml_.c_str());

            unsigned long long cur_ipoc = std::stoull(parsedXML.get("/Robot/Status@IPOC"));
            KukaRobotStateManager::getRobot(rob_id_).setOpMode( stoi(parsedXML.get("/Robot/Status@Mode")));

            std::string ctrlType = parsedXML.get("/Robot/Status@OpState");
            KukaRobotStateManager::getRobot(rob_id_).setRobOpState(ctrlType);

            KukaRobotState::KukaStatusCode curStatCode;
            curStatCode.code = 0;

            curStatCode.stat.Estop = (int)(1 == stoi(parsedXML.get("/Robot/Status@EStop")));
            curStatCode.stat.GuardStop = (int)(1 == stoi(parsedXML.get("/Robot/Status@GuardStop")));
            curStatCode.stat.DrivesPowered = (int)(1 == stoi(parsedXML.get("/Robot/Status@DrivesPowered")));
            curStatCode.stat.MotionPossible = (int)(1 == stoi(parsedXML.get("/Robot/Status@MotionPossible")));
            curStatCode.stat.InMotion = (int)(1 == stoi(parsedXML.get("/Robot/Status@InMotion")));
            curStatCode.stat.InError = (int)(1 == stoi(parsedXML.get("/Robot/Status@InError")));
            KukaRobotStateManager::getRobot(rob_id_).setStatus(curStatCode.stat);

            ROS_INFO_NAMED(logging_name_, "Set status: %u", curStatCode.code);

            int err = stoi(parsedXML.get("/Robot/Status@ErrorCode"));
            KukaRobotStateManager::getRobot(rob_id_).setErrorCode(err);

            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_Sstatus, cur_ipoc);
        }

        // Handle command acknowledge status changes
        if (messageType == "Ack")
        {
            messageHandled=true;
            std::string reqtype = parsedXML.get("/Robot/Ack@REQTYPE");
            unsigned long long ID = std::stoull(parsedXML.get("/Robot/Ack@ID"));
            // TODO: handle ack of commands after adding logging for tracking
        }

        // Handle the initialization message at initial connection
        if (messageType == "Init")
        {
            messageHandled=true;

            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

            // Store robot side version number.
            version_num_str_robot_ = parsedXML.get("/Robot/Init@VER");
            KukaRobotStateManager::getRobot(rob_id_).getRobotInfo().controller.sw_version = version_num_str_robot_;


            // Get the IPOC timestamp
            unsigned long long cur_ipoc = std::stoull(parsedXML.get("/Robot/Init@IPOC"));
            ROS_DEBUG_NAMED(logging_name_, "Init msg at IPOC= %llu", cur_ipoc);
            KukaRobotStateManager::getRobot(rob_id_).setIPOC(cur_ipoc);

            unsigned int numAxis = std::stoi(parsedXML.get("/Robot/Init@NumAx"));
            unsigned int numExtAxis = std::stoi(parsedXML.get("/Robot/Init@NumExAx"));
            KukaRobotStateManager::getRobot(rob_id_).setDOF(numAxis+numExtAxis);

            KukaRobotStateManager::getRobot(rob_id_).setRobName(parsedXML.get("/Robot/Init@Name"));
            KukaRobotStateManager::getRobot(rob_id_).setRobModel(parsedXML.get("/Robot/Init@Model"));

            std::string v = parsedXML.get("/Robot/Init@RobVer");
            // hardware and software version are separated by /
            size_t sep = v.find('/');
            std::string hw_ver = v.substr(0, sep);
            KukaRobotStateManager::getRobot(rob_id_).getRobotInfo().robots[0].hw_version =  hw_ver;
            std::string sw_ver = v.substr(sep+1);
            KukaRobotStateManager::getRobot(rob_id_).getRobotInfo().robots[0].sw_version =  sw_ver;

            KukaRobotStateManager::getRobot(rob_id_).setSerialNum(std::stoi(parsedXML.get("/Robot/Init@SerialNum")));


            // Add the supported features
            int hasFeature = 0;
            hasFeature = std::stoi(parsedXML.get("/Robot/Init/Features@EKI"));
            if (hasFeature == 1)
                KukaRobotStateManager::getRobot(rob_id_).addFeature(KukaRobotState::KukaCapability::KukaEKI);

            hasFeature = std::stoi(parsedXML.get("/Robot/Init/Features@RSI"));
            if (hasFeature == 1)
                KukaRobotStateManager::getRobot(rob_id_).addFeature(KukaRobotState::KukaCapability::KukaRSI);

            hasFeature = std::stoi(parsedXML.get("/Robot/Init/Features@MXA"));
            if (hasFeature == 1)
                KukaRobotStateManager::getRobot(rob_id_).addFeature(KukaRobotState::KukaCapability::KukaMxA);

            // Read the joint definition data
            for (int j=0; j<(numAxis+numExtAxis); j++)
            {
                std::string axisData = "/Robot/Init/Axis/" + makeAxisID(j);

                std::string typeXpath = axisData + "@Type";
                KukaRobotState::KukaJointType jtype = (KukaRobotState::KukaJointType)std::stoi(parsedXML.get(typeXpath));

                std::string numXpath = axisData + "@RatioNum";
                int numerator = std::stoi(parsedXML.get(numXpath));

                std::string denXpath = axisData + "@RatioDen";
                int denominator = std::stoi(parsedXML.get(denXpath));

                std::string rpmXpath = axisData + "@MaxRPM";
                double maxRPM = stod_safe(parsedXML.get(rpmXpath));

                KukaRobotStateManager::getRobot(rob_id_).setJointType(j, jtype);
                KukaRobotStateManager::getRobot(rob_id_).setJointGearRatio(j, numerator, denominator);
                KukaRobotStateManager::getRobot(rob_id_).setJointMotorMaxVel(j, maxRPM);
            }

            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInit, cur_ipoc);
            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitMod, cur_ipoc);
            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitName, cur_ipoc);
            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitRobV, cur_ipoc);
            // TODO: perform startup check task.
            // - Verify ROS model matches actual robot
            // - Verify axis count is correct
            // - Possibly expand data to include axis angle limits

        }

        // In case message type never got updated
        if (messageHandled==false)
        {
            ROS_ERROR_NAMED(logging_name_, "EKI XML unknown message type for message: %s", recv_xml_.c_str());
            return(false);
        }


    }
    catch (std::exception& ex)
    {
        ROS_ERROR_NAMED(logging_name_, "EKI XML problem extracting data: %s on %s", ex.what(), recv_xml_.c_str());
        throw(ex);
        return(false);
    }

    cycleState_ = CycleState::PARSED;
    // reset flag after data was parsed
    did_receive_msg_ = false;
    return(true);
}



std::string KukaCommHandlerEKI::makeAxisID(int j)
{
    char buf[8];
    std::string axisID;
    if ((0 <= j) && (j <= 5))
        sprintf(buf, "A%d", (j+1));
    else if ((6 <= j) && (j <= 11))
        sprintf(buf, "E%d", (j+1-6));
    else
        throw std::range_error("Invalid axis index in makeAxisID().");

    axisID = buf;
    return(axisID);
}




void KukaCommHandlerEKI::setXmlResponseTemplate(std::string xmlStr)
{
  response_XML_template_ = xmlStr;
  responseXML_->setXmlTemplate(response_XML_template_);
  req_type_response_str_ = kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_CmdJpos);
  responseXML_->set("/ROS@REQTYPE", (char *)req_type_response_str_.c_str());
}

std::string KukaCommHandlerEKI::getXmlResponseTemplate()
{
  return(response_XML_template_);
}



void KukaCommHandlerEKI::setXmlCommandTemplate(std::string xmlStr)
{
  command_XML_template_ = xmlStr;
  commandXML_->setXmlTemplate(command_XML_template_);
}

void KukaCommHandlerEKI::setXmlOpStateTemplate(std::string xmlStr)
{
  opState_XML_template_ = xmlStr;
  opStateXML_->setXmlTemplate(opState_XML_template_);
  req_type_opState_str_ = kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Change);
  opStateXML_->set("/ROS@REQTYPE", (char *)req_type_opState_str_.c_str());
}

bool KukaCommHandlerEKI::messagePrepareCommand(std::string commandID, unsigned long long ID)
{
    try {
        command_str_ = commandID;
        commandXML_->set("/ROS@REQTYPE", (char *)command_str_.c_str());
        id_str_ = std::to_string(ID);
        commandXML_->set("/ROS@ID", (char *)id_str_.c_str());

        std::lock_guard<std::mutex> lockBuff(buffer_mutex_);
        send_comm_buff_.append(commandXML_->asXml());
        cycleState_ = CycleState::PREPARED;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
        return(false);
    }
    return(true);
}

bool KukaCommHandlerEKI::messagePrepareOpState(std::string opStateID, unsigned long long ID)
{
    try {
        opState_str_ = opStateID;
        opStateXML_->set("/ROS/OpState@Type", (char *)opState_str_.c_str());
        id_str_ = std::to_string(ID);
        opStateXML_->set("/ROS@ID", (char *)id_str_.c_str());

        std::lock_guard<std::mutex> lockBuff(buffer_mutex_);
        send_comm_buff_.append(opStateXML_->asXml());
        cycleState_ = CycleState::PREPARED;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
        return(false);
    }
    return(true);
}


bool KukaCommHandlerEKI::messagePrepare(const ros::Duration period)
{
    duration_ = period.toSec();
    return(messagePrepare());
}


bool KukaCommHandlerEKI::messagePrepare()
{
    //TODO: add calcs for other control methods

    std::vector<double> cmd_corrections_;
    try{
        std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

        id_str_ = std::to_string(KukaRobotStateManager::getRobot(rob_id_).getIPOC());
        duration_str_ = std::to_string(duration_);
        cmd_corrections_str_[0] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(0));
        cmd_corrections_str_[1] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(1));
        cmd_corrections_str_[2] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(2));
        cmd_corrections_str_[3] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(3));
        cmd_corrections_str_[4] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(4));
        cmd_corrections_str_[5] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(5));
        cmd_corrections_str_[6] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(6));
        cmd_corrections_str_[7] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(7));
        cmd_corrections_str_[8] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(8));
        cmd_corrections_str_[9] = std::to_string( KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(9));
        cmd_corrections_str_[10] = std::to_string(KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(10));
        cmd_corrections_str_[11] = std::to_string(KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(11));

        //responseXML_->set("/ROS@REQTYPE", "CmdJpos-");
        responseXML_->set("/ROS@ID", (char *)id_str_.c_str());
        responseXML_->set("/ROS/Command@dt", (char *)duration_str_.c_str());
        responseXML_->set("/ROS/Command/Pos@A1", (char *)cmd_corrections_str_[0].c_str());
        responseXML_->set("/ROS/Command/Pos@A2", (char *)cmd_corrections_str_[1].c_str());
        responseXML_->set("/ROS/Command/Pos@A3", (char *)cmd_corrections_str_[2].c_str());
        responseXML_->set("/ROS/Command/Pos@A4", (char *)cmd_corrections_str_[3].c_str());
        responseXML_->set("/ROS/Command/Pos@A5", (char *)cmd_corrections_str_[4].c_str());
        responseXML_->set("/ROS/Command/Pos@A6", (char *)cmd_corrections_str_[5].c_str());
        responseXML_->set("/ROS/Command/Pos@E1", (char *)cmd_corrections_str_[6].c_str());
        responseXML_->set("/ROS/Command/Pos@E2", (char *)cmd_corrections_str_[7].c_str());
        responseXML_->set("/ROS/Command/Pos@E3", (char *)cmd_corrections_str_[8].c_str());
        responseXML_->set("/ROS/Command/Pos@E4", (char *)cmd_corrections_str_[9].c_str());
        responseXML_->set("/ROS/Command/Pos@E5", (char *)cmd_corrections_str_[10].c_str());
        responseXML_->set("/ROS/Command/Pos@E6", (char *)cmd_corrections_str_[11].c_str());

        std::lock_guard<std::mutex> lockBuff(buffer_mutex_);
        send_comm_buff_.append(responseXML_->asXml());
        //TODO - Investigate velocity settings in command.
//        ROS_INFO_THROTTLE_NAMED(0.5, logging_name_, "EKI::messagePrepare(): Vel=%f, %f, %f",
//                                KukaRobotStateManager::getRobot(rob_id_).getCommandJointVelocityPercent(0),
//                                KukaRobotStateManager::getRobot(rob_id_).getCommandJointVelocityPercent(1),
//                                KukaRobotStateManager::getRobot(rob_id_).getCommandJointVelocityPercent(2));
        cycleState_ = CycleState::PREPARED;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
        return(false);
    }
    return(true);
}

bool KukaCommHandlerEKI::messageSend()
{

    std::lock_guard<std::mutex> lockBuff(buffer_mutex_);
    size_t bytes_sent = comm_link_.send(send_comm_buff_);
    if ((bytes_sent > 0) && (send_comm_buff_.length() > 0) )
    {
        state_ = CONNECTED;
    }
    else
    {
        if (!comm_link_.is_connected())
        {
            state_ = DISCONNECTED;
            return(false);
        }
    }
    send_comm_buff_.clear();
    cycleState_ = CycleState::SENT;
    return(true);
}


ros::Duration KukaCommHandlerEKI::getCommCycle()
{
    ros::Duration dur;
    dur.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_received_ - t_received_last_).count());
    return(dur);
}


double KukaCommHandlerEKI::stod_safe(const char* num)
{
    double n = 0.0;
    try {
        n = std::stod(num);
    }
    catch (std::exception& ex) {
        n = 0.0;
    }
    return(n);
}


} //namespace kuka_kss_hw_interface

