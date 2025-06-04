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



#include <kuka_kss_hw_interface/comm_link_tcp_client.h>


namespace kuka_kss_hw_interface
{

CommLink_TCPClient::CommLink_TCPClient(const ros::NodeHandle& nh, const std::string log_id) :CommunicationLink(nh, log_id), local_client_socket_(ios_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0))
{
    receive_buffer.reserve(2048);
    tcp_timeout_.fromSec(5.0);  // [s]; settable by parameter (default = 5)
}

/**
 * @brief setupParams()
 * Default Required TCP Server ROS parameters:
 *    eki/robot_address
 *    eki/robot_port
 *
 * Optional ROS parameters:
 *    eki/socket_timeout - Time before connect fails
 * @return true
 */
bool CommLink_TCPClient::setupParams()
{
    return(setupParams("tcp/server_address", "tcp/server_port", "tcp/socket_timeout"));
}


bool CommLink_TCPClient::setupParams(std::string param_ip_addr, std::string param_port, std::string timeout)
{
    param_addr_ = param_ip_addr;
    param_port_ = param_port;
    param_timeout_ = timeout;
    return(true);
}


/**
 * @brief Setup()
 * Required TCP Server ROS parameters:
 *    eki/robot_address
 *    eki/robot_port
 *
 * Optional ROS parameters:
 *    eki/socket_timeout - Time before connect fails
 *
 * @return
 */
bool CommLink_TCPClient::setup()
{
    // Verify if parameters were set wiht setupParams.
    if (param_addr_.length() == 0)
    {
        // Use default params
        setupParams();
    }

    // Get TCP Server parameters from ROS
    if (nh_.getParam(param_addr_, server_address_) &&
        nh_.getParam(param_port_, server_port_))
    {
      ROS_INFO_STREAM_NAMED(get_logging_id(), "Configuring connection to Kuka TCP Server interface on: "
                            << server_address_ << ", " << server_port_);
    }
    else
    {
      std::string msg = "Failed to get TCP Server address/port from parameter server (looking for '" + param_addr_ +
                        "', '" + param_port_ + "')";
      ROS_ERROR_STREAM(msg);
      throw std::runtime_error(msg);
      return(false);
    }

    double timeout=5.0;
    if (nh_.getParam(param_timeout_, timeout))
    {
      ROS_INFO_STREAM_NAMED(get_logging_id(), "Configuring Kuka TCP hardware interface socket timeout to "
                            << timeout << " seconds");
      tcp_timeout_.fromSec(timeout);  // [s]; settable by parameter (default = 5)
    }
    else
    {
      ROS_INFO_STREAM_NAMED(get_logging_id(), "Failed to get TCP socket timeout from parameter server (looking "
                            "for '" + param_timeout_ + "'), defaulting to " +
                            std::to_string(tcp_timeout_.toSec())  + " seconds");
    }

    return(true);
}

bool CommLink_TCPClient::start()
{
    try {
        boost::asio::ip::tcp::resolver resolver(ios_);
        tcp_server_endpoint_ = *resolver.resolve({boost::asio::ip::tcp::v4(), server_address_, server_port_});
        local_client_socket_.connect(tcp_server_endpoint_);
        is_connected_ = local_client_socket_.is_open();
        if (is_connected_)
        {
            local_client_socket_.non_blocking(true);
            updateTimeStamp();
        }
    } catch (boost::system::system_error err) {
            is_connected_ = false;
            ROS_INFO_NAMED(get_logging_id(), "Kuka TCP hardware interface socket exception on open: %s", err.what());
    }
    return(is_connected_);
}

bool CommLink_TCPClient::stop()
{
    try {
        if (local_client_socket_.is_open())
        {
            local_client_socket_.close();
        }
        is_connected_ = local_client_socket_.is_open();
    } catch (boost::system::system_error err) {
            is_connected_ = false;
            ROS_INFO_NAMED(get_logging_id(), "Kuka TCP hardware interface socket exception on close: %s", err.what());
    }

    return(is_connected_);
}


size_t CommLink_TCPClient::receive(std::string& buffer)
{
    size_t len;
    boost::system::error_code ec = boost::asio::error::would_block;
    static boost::array<char, 2048> in_buffer;

    len = 0;
    // Non-blocking receive may have partial data but provides better real-time balancing.
    len = local_client_socket_.receive(boost::asio::buffer(in_buffer), 0, ec);
    if (len > 0)
    {
        updateTimeStamp();
    }

    buffer.assign(in_buffer.data(), len);
    return(len);
}

size_t CommLink_TCPClient::send(std::string& buffer)
{
    size_t len = local_client_socket_.send(boost::asio::buffer(buffer));
    if ((buffer.length() > 0) && (len > 0) )
    {
        updateTimeStamp();
    }

    return(len);
}

void CommLink_TCPClient::updateTimeStamp()
{
   t_last_comm_ = std::chrono::steady_clock::now();
}

bool CommLink_TCPClient::is_connected()
{
    std::chrono::time_point<std::chrono::steady_clock> cur_time;
    cur_time = std::chrono::steady_clock::now();

    ros::Duration elapsed;
    elapsed.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(cur_time - t_last_comm_).count());
    is_connected_ = (elapsed < tcp_timeout_);
    return (is_connected_);
}

ros::Duration CommLink_TCPClient::time_since_last_comm()
{
    std::chrono::time_point<std::chrono::steady_clock> cur_time;
    cur_time = std::chrono::steady_clock::now();

    ros::Duration elapsed;
    elapsed.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(cur_time - t_last_comm_).count());
    return (elapsed);
}



} //namespace kuka_kss_hw_interface


