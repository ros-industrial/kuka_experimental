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

#ifndef KUKA_COMM_HANDLER_H
#define KUKA_COMM_HANDLER_H

#include <string>
#include <stdexcept>
#include <ros/ros.h>
#include <kuka_kss_hw_interface/communication_link.h>
#include <kuka_kss_hw_interface/kuka_robot_state_manager.h>

namespace kuka_kss_hw_interface
{


/**
 * @brief A base class for a communication handling.  Implements communication handling and send/receive message protocol.
 *
 */
class KukaCommHandler
{
public:
    /**
     * @brief KukaCommHandler - A template class for kuka communication protocol handling.  Message receive should be non-blocking.
     * @param comm - The communication link mechanism.  Typically TCP, UDP.  Could be EtherCat or other fieldbus.
     * @param robot - The ID of the robot state object in the KukaRobotStateManager.
     * @param priority - A priority to manage processing order when multiple handlers are used.
     * @param nh - Node handle for accessing ROS methods.
     * @param log_id - The ROS logging name to use.
     */
    KukaCommHandler(CommunicationLink& comm, int rob_id, int priority, const ros::NodeHandle& nh, std::string log_id);
    virtual ~KukaCommHandler();

    enum KukaCommState
    {
        UNDEFINED = -1,
        INITIALIZED = 0,
        LISTENING = 1,
        CONNECTED = 2,
        DISCONNECTED = 3
    };

    /**
     * @brief setup - perform setup operations.  Typically init of parameters.
     * @return true if successful.
     */
    virtual bool setup() = 0;

    /**
     * @brief startComm - Start communication.
     * @return true if successful
     */
    virtual bool startComm() = 0;

    /**
     * @brief shutdownComm - Close down the communication link and perform shutdown state operations for disconnect.
     * @return
     */
    virtual bool shutdownComm() = 0;

    /**
     * @brief getCommState - The current communication state.
     * @return
     */
    KukaCommState getCommState() {return state_;}

    /**
     * @brief getPriority - The priority
     * @return
     */
    int getPriority() {return priority_;}

    /**
     * @brief checkForReceive - Checks the communication link in a non-blocking way to see if a message was received.
     * @return true=A complete message was received.
     */
    virtual bool checkForReceive() = 0;

    /**
     * @brief wasReceived - Checks the current state to see if a complete message is in buffer.
     * @return
     */
    virtual bool wasReceived() = 0;

    /**
     * @brief parseReceivedMessage - Interprets the message and reads the data into the robot state object.
     * @return true if successful
     */
    virtual bool parseReceivedMessage() = 0;

    /**
     * @brief messagePrepare - prepares a message to be sent.  Builds the protocol and stores in the send buffer.
     * @return true if successful
     */
    virtual bool messagePrepare() = 0;

    /**
     * @brief messageSend - Transmits the message.
     * @return true if successful
     */
    virtual bool messageSend() = 0;

protected:
    int rob_id_;
    const ros::NodeHandle& nh_;
    std::string logging_name_;
    KukaCommState state_;
    int priority_;
    CommunicationLink &comm_link_;

};


} //namespace kuka_kss_hw_interface


#endif // KUKA_COMM_HANDLER_H
