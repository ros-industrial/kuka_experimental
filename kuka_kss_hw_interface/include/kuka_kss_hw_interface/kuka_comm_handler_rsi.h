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

#ifndef KUKA_COMM_HANDLER_RSI_H
#define KUKA_COMM_HANDLER_RSI_H

#include <kuka_kss_hw_interface/kuka_comm_handler.h>
#include <kuka_kss_hw_interface/XmlTemplateXpath.h>


namespace kuka_kss_hw_interface
{

/**
 * @brief The KukaCommHandlerRSI class handles RSI communication to the Kuka
 * which uses a UDP Server for communication and XML formatted packets for data exchange
 * by the ETHERNET object in RSI.
 */
class KukaCommHandlerRSI : public KukaCommHandler
{
public:
    /**
     * @brief KukaCommHandlerRSI
     * @param comm - Must be a CommLink_UDPServer object.  Be sure to properly use setupParams before.
     * @param rob_id - the int ID of the robot state object in the KukaRobotStateManager
     * @param priority
     * @param nh
     * @param log_id
     */
    KukaCommHandlerRSI(CommunicationLink& comm, int rob_id, int priority, const ros::NodeHandle& nh, std::string log_id);
    virtual ~KukaCommHandlerRSI();

    virtual bool setup();
    virtual bool startComm();
    virtual bool shutdownComm();

    virtual bool checkForReceive();
    virtual bool wasReceived();
    virtual bool parseReceivedMessage();

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
     * @brief calcCommandCorrectionsPos calculate the RSI command values for position
     * control in RSI Absolute correction
     */
    void calcCommandCorrectionsPos();

    /**
     * @brief getDelay - returns the RSI Delay communication metric.  The number of delayed packets reported by the robot.
     * @return
     */
    int getDelay() const {return delay;}

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
     * @brief getParseTime - returns the recieve to parse complete time interval
     * @return
     */
    ros::Duration getParseTime();

    /**
     * @brief getUpdateTime - returns the interval from parse complete to start of messagePrepare
     * @return
     */
    ros::Duration getUpdateTime();

    /**
     * @brief getSendTime - returns the time interval from message prepare to send
     * @return
     */
    ros::Duration getSendTime();

    /**
     * @brief getResponseTime - The total response time from receive to send
     * @return
     */
    ros::Duration getResponseTime();


private:
    void setVersionInfo(std::string ver);
    std::string RSI_INTERFACE_VERSION_;
    std::string version_num_str_;
    std::string version_num_str_robot_;

    CycleState cycleState_;
    std::string startStr_ ;
    std::string endStr_ ;
    size_t endStr_len_;
    std::string recv_comm_buff_;
    std::string recv_msg_;
    std::string recv_xml_;
    bool did_receive_msg_;

    std::string send_comm_buff_;
    // Changing data in a pre-parsed XML structure generates faster
    std::string response_XML_template_;
    std::unique_ptr<XmlTemplateXpath> responseXML_;

    // RSI Specific robot state data
    bool did_init;
    // NOTE: RapidXML requires persistent strings for set values
    std::string ipoc_str_;
    std::string stop_str_;
    std::vector<std::string> cmd_corrections_str_;
    // RSI requires adjusted values based on starting position
    std::vector<double> cmd_corrections_;
    std::vector<double> initial_positions_;
    std::vector<double> cart_position_;
    std::vector<double> initial_cart_position_;
    int delay;

    // Track communication timing metrics
    std::chrono::time_point<std::chrono::steady_clock> t_received_last_;
    std::chrono::time_point<std::chrono::steady_clock> t_received_;
    std::chrono::time_point<std::chrono::steady_clock> t_parseComplete_;
    std::chrono::time_point<std::chrono::steady_clock> t_startPrepare_;
    std::chrono::time_point<std::chrono::steady_clock> t_sent_;

};

} //namespace kuka_kss_hw_interface

#endif // KUKA_COMM_HANDLER_RSI_H
