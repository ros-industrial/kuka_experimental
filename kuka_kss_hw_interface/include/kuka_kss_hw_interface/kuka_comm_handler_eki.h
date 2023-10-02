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

#ifndef KUKA_COMM_HANDLER_EKI_H
#define KUKA_COMM_HANDLER_EKI_H

#include <kuka_kss_hw_interface/kuka_comm_handler.h>
#include <kuka_kss_hw_interface/XmlTemplateXpath.h>


namespace kuka_kss_hw_interface
{

/**
 * @brief The KukaCommHandlerEKI class handles Ethernet KRL (EKI) communication to the Kuka
 * which uses a TCP Client for communication and XML formatted packets for data exchange.
 * The Kuka robot implements a TCP Server in the background SPS to handle communication.
 */
class KukaCommHandlerEKI : public KukaCommHandler
{
public:
    /**
     * @brief KukaCommHandlerEKI
     * @param comm - Must be a CommLink_UDPServer object.  Be sure to properly use setupParams before.
     * @param rob_id - the int ID of the robot state object in the KukaRobotStateManager
     * @param priority
     * @param nh
     * @param log_id
     */
    KukaCommHandlerEKI(CommunicationLink& comm, int rob_id, int priority, const ros::NodeHandle& nh, std::string log_id);
    virtual ~KukaCommHandlerEKI();

    virtual bool setup();
    virtual bool startComm();
    virtual bool shutdownComm();

    virtual bool checkForReceive();
    virtual bool wasReceived();
    virtual bool parseReceivedMessage();
    /**
     * @brief messagePrepareCommand - Prepare a custom command message to the robot.
     * @param commandID - Identifier string for the command type.
     * @param ID - Message ID number.  Use current IPOC value as standard (KukaRobotState::getIPOC() ).
     * @return
     */
    bool messagePrepareCommand(std::string commandID, unsigned long long ID);
    /**
     * @brief messagePrepareOpState
     * @param opStateID - use KukaRobotState::getControlModeStr(mode) to properly set the value
     * @param ID - Message ID number.  Use current IPOC value as standard (KukaRobotState::getIPOC() ).
     * @return
     */
    bool messagePrepareOpState(std::string opStateID, unsigned long long ID);
    bool messagePrepare(const ros::Duration period);
    virtual bool messagePrepare();
    virtual bool messageSend();

    enum CycleState
    {
        UNKNOWN=-1,
        INIT=0,
        WAITING=1,
        RECEIVING=2,
        RECEIVED=3,
        PARSED=4,
        PREPARED=5,
        SENT=6
    };
    /**
     * @brief getCommState returns the current state for the communication cycle
     * @return
     */
    CycleState getCycleState() {return cycleState_;}

    /**
     * @brief setXmlResponseTemplate - Predefine an XML template for messages to the robot.
     * A default template is already defined in the class
     * @param xmlStr
     */
    void setXmlResponseTemplate(std::string xmlStr);

    /**
     * @brief setXmlCommandTemplate - Predefine an XML template for commands to the robot.
     * A default template is already defined in the class
     * @param xmlStr
     */
    void setXmlCommandTemplate(std::string xmlStr);

    /**
     * @brief setXmlOpStateTemplate - Predefine an XML template for sending OpState command to the robot.
     * A default template is already defined in the class
     * @param xmlStr
     */
    void setXmlOpStateTemplate(std::string xmlStr);

    /**
     * @brief getXmlResponseTemplate - Get the template string.  Version data is set separately so
     * The template does not necessarily have the runtime version.
     * @return
     */
    std::string getXmlResponseTemplate();

    /**
     * @brief checkReceivedVersion - checks for a matching version in the received data.
     * parseReceivedMessage() must be called before this check.
     * @return
     */
    bool checkReceivedVersion();

    /**
     * @brief getReceivedXML - returns the full XML packet received from the robot.
     * @return
     */
    std::string getReceivedXML() const {return recv_xml_;}

    /**
     * @brief getCommCycle - returns the receive to receive time interval
     * @return
     */
    ros::Duration getCommCycle();

    /**
     * @brief makeAxisID - creates a string ID for Kuka standard axis names A1-A6, E1-E6
     * @param j - joint index 0-11
     * @return
     */
    std::string makeAxisID(int j);



private:
    /**
     * @brief stod_safe - performs stod() but catches exceptions for bad strings and returns default 0.0.
     * @param num - the double number string to convert
     * @return
     */
    double stod_safe(const char* num);
    void setVersionInfo(std::string ver);
    std::string EKI_INTERFACE_VERSION_;
    std::string version_num_str_;
    std::string version_num_str_robot_;

    std::mutex buffer_mutex_;
    CycleState cycleState_;
    std::string recv_comm_buff_;
    std::string recv_msg_;
    std::string recv_xml_;
    bool did_receive_msg_;

    std::string startStr_ ;
    std::string endStr_ ;
    size_t endStr_len_;

    std::string send_comm_buff_;
    // Changing data in a pre-parsed XML structure generates faster
    std::string response_XML_template_;
    std::unique_ptr<XmlTemplateXpath> responseXML_;

    std::string command_XML_template_;
    std::unique_ptr<XmlTemplateXpath> commandXML_;
    std::string command_str_;

    std::string opState_XML_template_;
    std::unique_ptr<XmlTemplateXpath> opStateXML_;
    std::string opState_str_;



    // EKI Specific robot state data
    bool did_init;
    // NOTE: RapidXML requires persistent strings for set values
    std::string id_strCMD_;
    std::string id_str_;
    std::string duration_str_;
    std::vector<std::string> cmd_corrections_str_;

    std::vector<double> cart_position_;
    double duration_;

    // Track communication timing metrics
    std::chrono::time_point<std::chrono::steady_clock> t_received_last_;
    std::chrono::time_point<std::chrono::steady_clock> t_received_;
};

} //namespace kuka_kss_hw_interface

#endif // KUKA_COMM_HANDLER_EKI_H
