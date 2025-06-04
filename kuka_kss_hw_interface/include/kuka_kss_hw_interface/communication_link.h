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

#ifndef KUKA_KSS_COMMUNICATION_LINK_H
#define KUKA_KSS_COMMUNICATION_LINK_H
#include <string>
#include <stdexcept>
#include <ros/ros.h>
#include <sys/types.h>


namespace kuka_kss_hw_interface
{
/**
 * @brief A base class for a communication point.
 * The receive method should be non-blocking.
 */
class CommunicationLink
{
public:
    //CommunicationLink(): is_ready_(false), is_connected_(false),has_data_(false) {}
    CommunicationLink(const ros::NodeHandle& nh, const std::string log_id);

    virtual ~CommunicationLink();

    /**
     * @brief Define the parameters for the communicaiton link.  This no parameter method can define default params but it
     * is recommended that an overloaded method is defined with arguments for the specific parameters needed.
     * @return true=success
     */
    virtual bool setupParams() = 0;

    /**
     * @brief Setup the communicaiton link.  Read parameters & prepare for connection.
     * @return true=success
     */
    virtual bool setup() = 0;

    /**
     * @brief Start the communication link.  Connect or prepare for communication.
     * @return
     */
    virtual bool start() = 0;

    /**
     * @brief Stop/disconnect the communication link.  Reset & prepare for reconnect.
     * @return
     */
    virtual bool stop() = 0;

    /**
     * @brief Try to receive data.  This should be non-blocking and return immediately.
     * @param buffer - the memory area where data will be returned.
     * @return  - the size of the data retrieved, typically bytes.  0 if nodata is present.
     */
    virtual size_t receive(std::string& buffer) = 0;

    /**
     * @brief Try to send data in the buffer.  Raises exception if send fails.
     * @param buffer - the data to send.
     * @return  - the size of the data sent, typically bytes.
     */
    virtual size_t send(std::string& buffer) = 0;

    /**
     * @brief Is ready for connection.  Normal result after setup()
     * @return
     */
    virtual bool is_ready() {return is_ready_;}

    /**
     * @brief If the communication link is established and ready to send/receive.
     * @return
     */
    virtual bool is_connected() {return is_connected_;}

    /**
     * @brief If data has been received and is ready to read with receive()
     * @return
     */
    virtual bool has_data() {return has_data_;}

    /**
     * @brief Helper function to return the name for ROS logging.
     * @return The string for named logging
     */
    const std::string get_logging_id() {return (logging_name_);}

protected:
    const ros::NodeHandle& nh_;
    bool has_data_;
    bool is_connected_;
    bool is_ready_;
    const std::string logging_name_;

};

} //namespace kuka_kss_hw_interface


#endif // KUKA_KSS_COMMUNICATION_LINK_H
