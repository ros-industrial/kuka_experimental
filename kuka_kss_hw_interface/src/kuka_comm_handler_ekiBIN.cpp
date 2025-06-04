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
 * Edited by Christopher Castillo <Christopher.Castillo@kuka.com>
 */



#include <kuka_kss_hw_interface/kuka_commands.h>
#include <kuka_kss_hw_interface/kuka_comm_handler_ekiBIN.h>
#include <vector>


namespace kuka_kss_hw_interface
{



KukaCommHandlerEKIBIN::KukaCommHandlerEKIBIN(CommunicationLink& comm, int id, int priority, const ros::NodeHandle& nh, std::string log_id):
    KukaCommHandler(comm, id, priority, nh, log_id),
    // Note: actual version is set below with setVersionInfo() in the constructor...
    EKI_INTERFACE_VERSION_("V0000ROS"), version_num_str_("1234"), version_num_str_robot_("1234"), did_init(false), cycleState_(UNKNOWN), did_receive_msg_(false),
    duration_(0.200)
{
    ROS_INFO_STREAM_NAMED(logging_name_, "Starting Kuka communication link.");
    t_received_ = std::chrono::steady_clock::now();
    t_received_last_ = t_received_;
    t_sent_ = t_received_;
    recv_msg_.clear();

    // Default end of receive message.
    endStr_ = "ENDROS";

    try {
        // Must be of the form VnnnnROS
        setVersionInfo("V2004ROS");
    } catch (std::exception& ex){
        ROS_ERROR_STREAM_NAMED(logging_name_, "Error setting response XML.");
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
    }
    //ROS_INFO_STREAM_NAMED(logging_name_, "Completed KukaCommHandlerEKI constructor.");
}


KukaCommHandlerEKIBIN::~KukaCommHandlerEKIBIN()
{

}

void KukaCommHandlerEKIBIN::setVersionInfo(std::string ver)
{
    EKI_INTERFACE_VERSION_ = ver;
    // Extract the version number from format VnnnnROS
    version_num_str_ = EKI_INTERFACE_VERSION_.substr(1,4);

    ROS_INFO_NAMED(logging_name_,"Kuka ROS EKI BIN interface version %s.", EKI_INTERFACE_VERSION_.c_str());
}




bool KukaCommHandlerEKIBIN::checkReceivedVersion()
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


bool KukaCommHandlerEKIBIN::setup()
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

bool KukaCommHandlerEKIBIN::startComm()
{
    bool started;
    started = comm_link_.start();
    if (started)
    {
        state_ = LISTENING;
        cycleState_ = CycleState::WAITING;
        kuka_commands& cmdNames = kuka_commands::getInstance();
        cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInit, 0);
        cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitMod, 0);
        cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitName, 0);
        cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitRobV, 0);
        cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_Sstatus, 0);
        cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SjState, 0);
        ROS_INFO_STREAM_NAMED(logging_name_, "Started Kuka EKI communication link.");
    }
    else
    {
        ROS_WARN_STREAM_NAMED(logging_name_, "Kuka EKI startComm connection unsuccessful.");
    }
    return(started);
}


bool KukaCommHandlerEKIBIN::shutdownComm()
{
    bool success = comm_link_.stop();

    state_ = LISTENING;
    cycleState_ = CycleState::INIT;
    did_init = false;
    ROS_INFO_STREAM_NAMED(logging_name_, "Stopped Kuka EKI communication link.");
    return(success);
}

bool KukaCommHandlerEKIBIN::checkForReceive()
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

//    size_t endStrPos;
//    // A quick search for the closing char sequence
//    endStrPos = recv_msg_.find(endStr_);
//    // not found
//    if (endStrPos == std::string::npos)
//    {
//        //std::cout << "Partial packet." << std::endl;
//        ROS_DEBUG_NAMED(logging_name_, "Partial packet: %s ", recv_msg_.c_str());
//        did_receive_msg_ = false;
//        return(did_receive_msg_);
//    }

    cycleState_ = CycleState::RECEIVED;
    t_received_last_ = t_received_;
    t_received_ = std::chrono::steady_clock::now();
    did_receive_msg_ = true;
    recv_bin_.clear();
    recv_bin_ = recv_msg_;
    // Remove the packet from the receive_buffer
    recv_msg_.clear();
    ROS_DEBUG_NAMED(logging_name_, "Ready to process: %s ", recv_bin_.c_str());

    return(did_receive_msg_);
}

bool KukaCommHandlerEKIBIN::wasReceived()
{
    return(did_receive_msg_);
}

#define messageTypeSize 8
#define opStateStrSize 64
#define RobNameStrSize 50
#define RobModelStrSize 32
#define RobVersionStrSize 32
bool KukaCommHandlerEKIBIN::parseReceivedMessage()
{
    if (!did_receive_msg_)
    {
//        ROS_DEBUG_STREAM_NAMED(logging_name_, "Attempt to parse before received message.");
        return(false);
    }

    char messageType[messageTypeSize+1];
    recvAck_ = false;
    char* parsePtr = &recv_bin_[0];


    try {
        //read the message type
        std::memcpy(&messageType, parsePtr, messageTypeSize*sizeof(char));
        parsePtr += messageTypeSize*sizeof(char);
        messageType[messageTypeSize] = '\0';

        str_messageType_ = messageType;
        //ROS_DEBUG_NAMED(logging_name_, "Recognized message type: %s", str_messageType_.c_str());

    } catch (std::exception& ex) {
        ROS_ERROR_NAMED(logging_name_, "EKI BIN problem extracting message type. Exception: %s", ex.what());
        throw(ex);
        return(false);
    }


    try {
        bool messageHandled=false;
        kuka_commands& cmdNames = kuka_commands::getInstance();

        // Handle the robot position state update
        if (str_messageType_ == cmdNames[kuka_commands::CommandType::ROSC_SjState])
        {
            //ROS_DEBUG_NAMED(logging_name_, "JOINT STATE MSG RECEIVED");
            recvAck_ = true;
            messageHandled=true;
            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

            // Get the IPOC timestamp
            unsigned long long cur_ipoc;
            int int_ipoc;
            std::memcpy(&int_ipoc, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            cur_ipoc = static_cast<unsigned long long>(int_ipoc);
            ROS_DEBUG_NAMED(logging_name_, "State msg at IPOC= %llu", cur_ipoc);
            //uncommented for testing
            KukaRobotStateManager::getRobot(rob_id_).setIPOC(cur_ipoc);

            // Extract axis specific actual position =========================================
            // External axes
            std::vector<double> testRecvPos(12);
            float tempRecv;
            for(int j = 0; j < 12; ++j)
            {
                std::memcpy(&tempRecv, parsePtr, sizeof(float));
                parsePtr += sizeof(float);
                testRecvPos[j] = static_cast<double>(tempRecv);
                KukaRobotStateManager::getRobot(rob_id_).setJointPosition(j, testRecvPos[j]);
            }


            // Extract axis specific velocity =========================================
            // External axes
            std::vector<double> testRecvVel(12);
            for(int j = 0; j < 12; ++j)
            {
                std::memcpy(&tempRecv, parsePtr, sizeof(float));
                parsePtr += sizeof(float);
                testRecvVel[j] = static_cast<double>(tempRecv);
                KukaRobotStateManager::getRobot(rob_id_).setJointVelocity(j, testRecvVel[j]);
            }

            // Extract axis specific effort =========================================
            // External axes
            std::vector<double> testRecvEff(12);
            for(int j = 0; j < 12; ++j)
            {
                std::memcpy(&tempRecv, parsePtr, sizeof(float));
                parsePtr += sizeof(float);
                testRecvEff[j] = static_cast<double>(tempRecv);
                KukaRobotStateManager::getRobot(rob_id_).setJointEffort(j, testRecvEff[j]);
            }

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
        if (str_messageType_ == cmdNames[kuka_commands::CommandType::ROSC_Sstatus])
        {
            messageHandled=true;
            //ROS_INFO_NAMED(logging_name_, "STATUS MESSAGE RECEIVED");
            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getStatusMutex());

            // Get the IPOC timestamp
            unsigned long long cur_ipoc;
            int int_ipoc;
            std::memcpy(&int_ipoc, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            cur_ipoc = static_cast<unsigned long long>(int_ipoc);

//            KukaRobotStateManager::getRobot(rob_id_).setOpMode( stoi(parsedXML.get("/Robot/Status@Mode")));
            int opMode;
            std::memcpy(&opMode, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            KukaRobotStateManager::getRobot(rob_id_).setOpMode(opMode);

            KukaRobotState::KukaStatusCode curStatCode;
            curStatCode.code = 0;

            int value;
//            curStat.Estop = (int)(1 == stoi(parsedXML.get("/Robot/Status@EStop")));
            std::memcpy(&value, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            curStatCode.stat.Estop = (int)(1 == value);
//            curStat.GuardStop = (int)(1 == stoi(parsedXML.get("/Robot/Status@GuardStop")));
            std::memcpy(&value, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            curStatCode.stat.GuardStop = (int)(1 == value);
//            curStat.DrivesPowered = (int)(1 == stoi(parsedXML.get("/Robot/Status@DrivesPowered")));
            std::memcpy(&value, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            curStatCode.stat.DrivesPowered = (int)(1 == value);
//            curStat.MotionPossible = (int)(1 == stoi(parsedXML.get("/Robot/Status@MotionPossible")));
            std::memcpy(&value, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            curStatCode.stat.MotionPossible = (int)(1 == value);
//            curStat.InMotion = (int)(1 == stoi(parsedXML.get("/Robot/Status@InMotion")));
            std::memcpy(&value, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            curStatCode.stat.InMotion = (int)(1 == value);
//            curStat.InError = (int)(1 == stoi(parsedXML.get("/Robot/Status@InError")));
            std::memcpy(&value, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            curStatCode.stat.InError = (int)(1 == value);

            KukaRobotStateManager::getRobot(rob_id_).setStatus(curStatCode.stat);

            //            int err = stoi(parsedXML.get("/Robot/Status@ErrorCode"));
            std::memcpy(&value, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            int err = value;
            KukaRobotStateManager::getRobot(rob_id_).setErrorCode(err);

//            std::string ctrlType = parsedXML.get("/Robot/Status@OpState");
            char opState[opStateStrSize+1];
            std::memcpy(opState, parsePtr, opStateStrSize*sizeof(char));
            parsePtr += opStateStrSize*sizeof(char);
            std::string ctrlType = opState;
            KukaRobotStateManager::getRobot(rob_id_).setRobOpState(ctrlType);

            ROS_DEBUG_NAMED(logging_name_, "Status data: Mode=%d EStop=%d GuardStop=%d DrivesPowered=%d MotionPossible=%d InMotion=%d InError=%d ErrorCode=%d  OpState=%s",
                                            opMode, curStatCode.stat.Estop, curStatCode.stat.GuardStop, curStatCode.stat.DrivesPowered, curStatCode.stat.MotionPossible, curStatCode.stat.InMotion, curStatCode.stat.InError, err, ctrlType.c_str());
            ROS_INFO_NAMED(logging_name_, "Set status: %u", curStatCode.code);

            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_Sstatus, cur_ipoc);
        }

        // Handle command acknowledge status changes
        if (str_messageType_ == cmdNames[kuka_commands::CommandType::ROSC_Sackn])
        {
            char ackMessageType[messageTypeSize+1];
            messageHandled=true;

            std::memcpy(&ackMessageType, parsePtr, messageTypeSize*sizeof(char));
            parsePtr += messageTypeSize*sizeof(char);
            ackMessageType[messageTypeSize] = '\0';

            recvAck_ = true;
            std::memcpy(&ackID_, parsePtr, sizeof(int));
            parsePtr += sizeof(int);

            //ROS_DEBUG_NAMED(logging_name_, "ACKNOWLEDGE MESSAGE RECEIVED cmd=%s, id=%d", ackMessageType, ackID_);
            // TODO: handle ack of commands after adding logging for tracking
        }

        // Handle the initialization data message at initial connection
        if (str_messageType_ == cmdNames[kuka_commands::CommandType::ROSC_SInit])
        {
            messageHandled=true;

            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

            // Store robot side version number.
            std::memcpy(&version_num_robot_, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            version_num_str_robot_ = std::to_string(version_num_robot_);
            KukaRobotStateManager::getRobot(rob_id_).getRobotInfo().controller.sw_version = version_num_str_robot_;


            // Get the IPOC timestamp
            int int_ipoc;
            std::memcpy(&int_ipoc, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            unsigned long long cur_ipoc = static_cast<unsigned long long>(int_ipoc);
            ROS_INFO_NAMED(logging_name_, "Init msg at IPOC= %llu", cur_ipoc);
            KukaRobotStateManager::getRobot(rob_id_).setIPOC(cur_ipoc);

            // Get the number of axes
            unsigned int numAxis, numExtAxis;
            std::memcpy(&numAxis, parsePtr, sizeof(int));
            parsePtr += sizeof(int);

            std::memcpy(&numExtAxis, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            KukaRobotStateManager::getRobot(rob_id_).setDOF(numAxis+numExtAxis);

            // Get robot serial number
            int serialNum;
            std::memcpy(&serialNum, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            KukaRobotStateManager::getRobot(rob_id_).setSerialNum(serialNum);

            // Add the supported features ---------------------------
            int hasFeatureEKI = 0;
            // Supports EKI
            std::memcpy(&hasFeatureEKI, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            if (hasFeatureEKI == 1)
                KukaRobotStateManager::getRobot(rob_id_).addFeature(KukaRobotState::KukaCapability::KukaEKI);

            // Supports RSI
            int hasFeatureRSI = 0;
            std::memcpy(&hasFeatureRSI, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            if (hasFeatureRSI == 1)
                KukaRobotStateManager::getRobot(rob_id_).addFeature(KukaRobotState::KukaCapability::KukaRSI);

            // Supports mxAuto
            int hasFeatureMxA = 0;
            std::memcpy(&hasFeatureMxA, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            if (hasFeatureMxA == 1)
                KukaRobotStateManager::getRobot(rob_id_).addFeature(KukaRobotState::KukaCapability::KukaMxA);

            // Read the joint type definition data
            std::vector<int> axisTypes(12);
            for (int j=0; j<12; j++)
            {
                std::memcpy(&axisTypes[j], parsePtr, sizeof(int));
                parsePtr += sizeof(int);
            }

            //read joint ratio numerators
            std::vector<int> axisNumerator(12);
            for (int j = 0; j<12; j++)
            {
                std::memcpy(&axisNumerator[j], parsePtr, sizeof(int));
                parsePtr += sizeof(int);
            }

            //read joint ratio denominators
            std::vector<int> axisDenominator(12);
            for (int j = 0; j < 12; j++)
            {
                std::memcpy(&axisDenominator[j], parsePtr, sizeof(int));
                parsePtr += sizeof(int);
            }

            //read joint max rpm
            std::vector<float> axisMaxRpm(12);
            for (int j = 0; j < 12; j++)
            {
                std::memcpy(&axisMaxRpm[j], parsePtr, sizeof(float));
                parsePtr += sizeof(float);
            }

            // Set the values - Only do the axes that are actually used
            for (int j = 0; j < (numAxis+numExtAxis); j++)
            {
                KukaRobotStateManager::getRobot(rob_id_).setJointType(j, (KukaRobotState::KukaJointType)axisTypes[j]);
                KukaRobotStateManager::getRobot(rob_id_).setJointGearRatio(j, axisNumerator[j], axisDenominator[j]);
                KukaRobotStateManager::getRobot(rob_id_).setJointMotorMaxVel(j, axisMaxRpm[j]);
            }

            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInit, cur_ipoc);

            // TODO: perform startup check task.
            // - Verify ROS model matches actual robot
            // - Verify axis count is correct
            // - Possibly expand data to include axis angle limits

        }

        // Handle the initialization data message at initial connection - Robot Name
        if (str_messageType_ == cmdNames[kuka_commands::CommandType::ROSC_SInitName])
        {
            messageHandled=true;

            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

            //ROS_DEBUG_NAMED(logging_name_, "INIT MSG RECEIVED");
            // Get the IPOC timestamp
            unsigned long long cur_ipoc;
            int int_ipoc;
            std::memcpy(&int_ipoc, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            cur_ipoc = static_cast<unsigned long long>(int_ipoc);

            char name[RobNameStrSize+1];
            std::memcpy(name, parsePtr, RobNameStrSize*sizeof(char));
            parsePtr += RobNameStrSize*sizeof(char);
            std::string robName = name;
            KukaRobotStateManager::getRobot(rob_id_).setRobName(robName);

            //ROS_INFO_NAMED(logging_name_, "InitName msg at IPOC= %llu", cur_ipoc);
            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitName, cur_ipoc);
        }


        // Handle the initialization data message at initial connection - Robot Model
        if (str_messageType_ == cmdNames[kuka_commands::CommandType::ROSC_SInitMod])
        {
            messageHandled=true;

            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());
            // Get the IPOC timestamp
            unsigned long long cur_ipoc;
            int int_ipoc;
            std::memcpy(&int_ipoc, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            cur_ipoc = static_cast<unsigned long long>(int_ipoc);

            //ROS_DEBUG_NAMED(logging_name_, "INIT MSG RECEIVED");

            char model[RobModelStrSize+1];
            std::memcpy(model, parsePtr, RobModelStrSize*sizeof(char));
            parsePtr += RobModelStrSize*sizeof(char);
            std::string robModel = model;
            KukaRobotStateManager::getRobot(rob_id_).setRobModel(robModel);

            //ROS_INFO_NAMED(logging_name_, "InitMod msg at IPOC= %llu", cur_ipoc);
            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitMod, cur_ipoc);
        }

        // Handle the initialization data message at initial connection - Robot Version
        if (str_messageType_ == cmdNames[kuka_commands::CommandType::ROSC_SInitRobV])
        {
            messageHandled=true;

            std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());
            // Get the IPOC timestamp
            unsigned long long cur_ipoc;
            int int_ipoc;
            std::memcpy(&int_ipoc, parsePtr, sizeof(int));
            parsePtr += sizeof(int);
            cur_ipoc = static_cast<unsigned long long>(int_ipoc);

            //ROS_DEBUG_NAMED(logging_name_, "INIT MSG RECEIVED");

            char version[RobModelStrSize+1];
            std::memcpy(version, parsePtr, RobModelStrSize*sizeof(char));
            parsePtr += RobModelStrSize*sizeof(char);
            std::string v = version;

            // hardware and software version are separated by /
            size_t sep = v.find('/');
            std::string hw_ver = v.substr(0, sep);
            KukaRobotStateManager::getRobot(rob_id_).getRobotInfo().robots[0].hw_version =  hw_ver;
            std::string sw_ver = v.substr(sep+1);
            KukaRobotStateManager::getRobot(rob_id_).getRobotInfo().robots[0].sw_version =  sw_ver;

            //ROS_INFO_NAMED(logging_name_, "InitRobV msg at IPOC= %llu", cur_ipoc);
            cmdNames.setTimestamp(kuka_commands::CommandType::ROSC_SInitRobV, cur_ipoc);
        }


        // In case message type never got updated
        if (messageHandled==false)
        {
            ROS_ERROR_NAMED(logging_name_, "EKI BIN unknown message type for message: %s", recv_bin_.c_str());

            return(false);
        }


    }
    catch (std::exception& ex)
    {
        ROS_ERROR_NAMED(logging_name_, "EKI BIN problem extracting data: %s", ex.what());
        throw(ex);
        return(false);
    }

    cycleState_ = CycleState::PARSED;
    // reset flag after data was parsed
    did_receive_msg_ = false;
    return(true);
}



bool KukaCommHandlerEKIBIN::messagePrepareCommand(std::string commandID, unsigned long long ID)
{
    try {
        command_str_ = commandID;

        std::lock_guard<std::mutex> lockBuff(buffer_mutex_);

        ROS_DEBUG_NAMED(logging_name_, "messagePrepareCommand: %s", commandID.c_str());

        std::string id_str;
        id_str.resize(sizeof(int));
        char* ptr = &id_str[0];

        int cmdID = static_cast<int>(ID);
        std::memcpy(ptr, &cmdID, sizeof(int));
        ptr += sizeof(int);

        send_comm_buff_.append(commandID);
        send_comm_buff_.append(id_str);
        send_comm_buff_.append(endStr_);

        cycleState_ = CycleState::PREPARED;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
        return(false);
    }
    return(true);
}

bool KukaCommHandlerEKIBIN::messagePrepareOpState(std::string opStateID, unsigned long long ID)
{
    try {
        ROS_DEBUG_NAMED(logging_name_, "messagePrepareOpState: %s", opStateID.c_str());

        opState_str_ = opStateID;
        //opStateXML_->set("/ROS/OpState@Type", (char *)opState_str_.c_str());

        //commandXML_->set("/ROS@ID", (char *)id_str_.c_str());
        std::string id_str;
        id_str.resize(sizeof(int));
        char* ptr = &id_str[0];

        int cmdID = static_cast<int>(ID);
        std::memcpy(ptr, &cmdID, sizeof(int));
        ptr += sizeof(int);

        // Command name
        std::string cmdNameStr = kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_Change);

        std::lock_guard<std::mutex> lockBuff(buffer_mutex_);
        send_comm_buff_.append(cmdNameStr);
        send_comm_buff_.append(id_str);
        send_comm_buff_.append(opState_str_);
        //send_comm_buff_.append(endStr_);
        cycleState_ = CycleState::PREPARED;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
        return(false);
    }
    return(true);
}


bool KukaCommHandlerEKIBIN::messagePrepare(const ros::Duration period)
{
    duration_ = period.toSec();
    return(messagePrepare());
}


bool KukaCommHandlerEKIBIN::messagePrepare()
{
    //TODO: add calcs for other control methods

//    std::vector<double> cmd_corrections_;

    try{
        std::lock_guard<std::mutex> lock(KukaRobotStateManager::getRobot(rob_id_).getRobotMutex());

        //create temp string
        std::string jointPos_binStr;
        jointPos_binStr.resize(sizeof(int) + sizeof(float) + 36 * sizeof(float) + 1);
        char* ptr = &jointPos_binStr[0];

        int cmdID = static_cast<int>(KukaRobotStateManager::getRobot(rob_id_).getIPOC());
        std::memcpy(ptr, &cmdID, sizeof(int));
        ptr += sizeof(int);

        //copy duration to bin str
//        std::cout << "Writing dTime to buffer: " << duration_ << std::endl;
        float dfTime = static_cast<float>(duration_);
        std::memcpy(ptr, &dfTime, sizeof(float));
        ptr += sizeof(float);

        //copy position data to bin str
        for(int j=0; j<12; j++)
        {
            float val = static_cast<float>(KukaRobotStateManager::getRobot(rob_id_).getCommandJointPositionKuka(j));
            std::memcpy(ptr, &val, sizeof(float));
            ptr += sizeof(float);
        }

        //copy velocity data to bin str
        for(int j=0; j<12; j++)
        {
            float val = static_cast<float>(KukaRobotStateManager::getRobot(rob_id_).getCommandJointVelocityPercent(j));
            std::memcpy(ptr, &val, sizeof(float));
            ptr += sizeof(float);
        }

        // TODO - integrate actual effort values
        std::vector<float> testEffData = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f, 0.09f, 0.10f, 0.11f, 0.12f};
        //copy effort data to bin str
        for(int j=0; j<12; j++)
        {
            float val = testEffData[j];
            std::memcpy(ptr, &val, sizeof(float));
            ptr += sizeof(float);
        }

        // Command name
        std::string cmdNameStr = kuka_commands::getInstance().getCommand(kuka_commands::CommandType::ROSC_CmdJpos);

        std::lock_guard<std::mutex> lockBuff(buffer_mutex_);
        //append bin str & end str to send buffer
        send_comm_buff_.append(cmdNameStr);
        send_comm_buff_.append(jointPos_binStr);
        send_comm_buff_.append(endStr_);

        cycleState_ = CycleState::PREPARED;
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(logging_name_, ex.what());
        return(false);
    }
    return(true);
}

bool KukaCommHandlerEKIBIN::messageSend()
{

    std::string bin_buff;

    std::lock_guard<std::mutex> lockBuff(buffer_mutex_);
    if (send_comm_buff_.size() > 256)
    {
        ROS_INFO_NAMED(logging_name_, "KukaCommHandlerEKIBIN::messageSend() large buffer size=%d", (int)send_comm_buff_.size());
    }

    // Handle multiple messages queued in send_comm_buff_
    while (send_comm_buff_.size() > 0)
    {
        if (send_comm_buff_.size() < 256)
        {
            send_comm_buff_.resize(256);
        }
        bin_buff = send_comm_buff_.substr(0, 256);
        bin_buff.resize(256);

        size_t bytes_sent = comm_link_.send(bin_buff);
        t_sent_ = std::chrono::steady_clock::now();

        if ((bytes_sent > 0) && (send_comm_buff_.size() > 0) )
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
        send_comm_buff_.erase(0, 256);
    }
    cycleState_ = CycleState::SENT;
    return(true);
}


ros::Duration KukaCommHandlerEKIBIN::getCommCycle()
{
    ros::Duration dur;
    dur.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_received_ - t_received_last_).count());
    return(dur);
}

ros::Duration KukaCommHandlerEKIBIN::timeoutCheck()
{
    ros::Duration timeoutCheck;
    timeoutCheck.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_received_last_).count());
    return(timeoutCheck);
}

bool KukaCommHandlerEKIBIN::checkAck()
{
    return recvAck_;
}

double KukaCommHandlerEKIBIN::stod_safe(const char* num)
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

