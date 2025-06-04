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


#include <kuka_kss_hw_interface/comm_link_udp_server.h>


namespace kuka_kss_hw_interface
{

CommLink_UDPServer::CommLink_UDPServer(const ros::NodeHandle& nh, const std::string log_id) :CommunicationLink(nh, log_id),
    local_server_socket_(ios_)
{
    receive_buffer.reserve(2048);
}


bool CommLink_UDPServer::setupParams()
{
    return(setupParams("udp/listen_port", "udp/listen_address"));
}


bool CommLink_UDPServer::setupParams(std::string param_ip_addr, std::string param_port)
{
    param_addr_ = param_ip_addr;
    param_port_ = param_port;
    return(true);
}


bool CommLink_UDPServer::setup()
{
    if (param_port_.length() == 0)
    {
        // Use default params
        setupParams();
    }

    //TODO: handle no server address param - use port on default interface.
    // Get UDP Server parameters from ROS
    if (nh_.getParam(param_addr_, local_server_address_) &&
        nh_.getParam(param_port_, local_server_port_))
    {
      ROS_INFO_STREAM_NAMED(get_logging_id(), "Configuring ROS UDP Server interface on: "
                            << local_server_address_ << ", " << local_server_port_);
    }
    else
    {
      std::string msg = "Failed to get UDP Server address/port from parameter server (looking for '" + param_addr_ +
                        "', '" + param_port_ + "')";
      ROS_ERROR_STREAM(msg);
      throw std::runtime_error(msg);
      return(false);
    }

    return(true);
}

bool CommLink_UDPServer::start()
{
    try {
        local_server_socket_.open(boost::asio::ip::udp::v4());

        //int svr_port = std::stoi(local_server_port_);
        //local_server_endpoint_.port(local_server_port_);
        local_server_endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_server_port_);

        boost::system::error_code ec = boost::asio::error::fault;

        local_server_socket_.bind(local_server_endpoint_, ec);
        ROS_INFO_NAMED(get_logging_id(), "Kuka Server socket bind ec=%s", ec.message().c_str());

        local_server_socket_.non_blocking(true);

        ROS_INFO_NAMED(get_logging_id(), "Kuka UDP Server listening on port %d", local_server_port_);
        is_connected_ = local_server_socket_.is_open();
    } catch (boost::system::system_error err) {
        is_connected_ = false;
        ROS_INFO_NAMED(get_logging_id(), "Kuka UDP hardware interface socket exception on open: %s", err.what());
    }
    return(is_connected_);
}

bool CommLink_UDPServer::stop()
{
    try {
        if (local_server_socket_.is_open())
        {
            local_server_socket_.close();
        }
        is_connected_ = local_server_socket_.is_open();
    } catch (boost::system::system_error err) {
            is_connected_ = false;
            ROS_INFO_NAMED(get_logging_id(), "Kuka UDP hardware interface socket exception on close: %s", err.what());
    }

    return(is_connected_);
}

bool CommLink_UDPServer::is_ready()
{
    is_ready_ = local_server_socket_.is_open();
    return(is_ready_);
}

bool CommLink_UDPServer::is_connected()
{
    is_connected_ = true;
    boost::system::error_code ec;
    boost::asio::ip::udp::endpoint endpoint = local_server_socket_.remote_endpoint(ec);
    if (ec == boost::asio::error::connection_aborted)
        is_connected_ = false;
    if (endpoint.address().is_unspecified())
        is_connected_ = false;
    return(is_connected_);
}


bool CommLink_UDPServer::has_data()
{
    has_data_ = (local_server_socket_.available() > 0);
    return has_data_;
}


size_t CommLink_UDPServer::receive(std::string& buffer)
{
    size_t len;
    boost::system::error_code ec = boost::asio::error::would_block;
    static boost::array<char, 2048> in_buffer;

    len = 0;
    len = local_server_socket_.receive_from(boost::asio::buffer(in_buffer), remote_endpoint_, 0, ec);

    buffer.assign(in_buffer.data(), len);
    return(len);
}

size_t CommLink_UDPServer::send(std::string& buffer)
{
    size_t len = local_server_socket_.send_to(boost::asio::buffer(buffer), remote_endpoint_);
    return(len);
}




} //namespace kuka_kss_hw_interface


