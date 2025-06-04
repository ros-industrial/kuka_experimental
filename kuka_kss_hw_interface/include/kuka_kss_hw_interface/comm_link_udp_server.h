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

#ifndef KUKA_KSS_COMM_LINK_UDP_SERVER_H
#define KUKA_KSS_COMM_LINK_UDP_SERVER_H
#include <string>
#include <stdexcept>
#include <kuka_kss_hw_interface/communication_link.h>

#include <boost/chrono/duration.hpp>
#include <boost/asio.hpp>


namespace kuka_kss_hw_interface
{
/**
 * @brief Implementation for a UDP Communication Server.
 * Based on Boost asio for platform portability.
 */
class CommLink_UDPServer : public CommunicationLink
{
public:
    CommLink_UDPServer(const ros::NodeHandle& nh, const std::string log_id);

    /**
     * @brief setupParams - Setup with the default parameters.
     * @return true if the parameters are found.
     */
    virtual bool setupParams();

    /**
     * @brief setupParams - Setup the parameters that the class will look at.
     * @param param_ip_addr - The ROS param for IPV4 address.
     * @param param_port - The ROS param for the integer socket port.
     * @return true if the parameters are found.
     */
    bool setupParams(std::string param_ip_addr, std::string param_port);

    /**
     * @brief Setup the communicaiton link.  Read parameters & prepare for connection.
     * @return true=success
     */
    virtual bool setup();

    /**
     * @brief Start the communicatin link.  Connect or prepare for communication.
     * @return
     */
    virtual bool start();

    /**
     * @brief Stop/disconnect the communication link.  Reset & prepare for reconnect.
     * @return
     */
    virtual bool stop();

    /**
     * @brief Try to receive data.  Will return immediately (non-blocking).
     * @param buffer - the memory area where data will be returned.
     * @return  - the size of the data retrieved, typically bytes. 0 if no data.
     */
    virtual size_t receive(std::string& buffer);

    /**
     * @brief Try to send data in the buffer.  Raises exception if send fails.
     * @param buffer - the data to send.
     * @return  - the size of the data sent, typically bytes.
     */
    virtual size_t send(std::string& buffer);

    virtual bool has_data();

    virtual bool is_connected();

    virtual bool is_ready();

private:
    boost::asio::io_service ios_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::asio::ip::udp::endpoint local_server_endpoint_;
    boost::asio::ip::udp::socket local_server_socket_;
    std::string param_addr_;
    std::string local_server_address_;
    std::string param_port_;
    int local_server_port_;
    std::string receive_buffer;


};

} //namespace kuka_kss_hw_interface


#endif // KUKA_KSS_COMM_LINK_UDP_SERVER_H
