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


#include <string>
#include <kuka_msgs/kuka_msgs_util.h>


namespace kuka_msgs
{


using namespace std;

KukaMsgsUtil::KukaMsgsUtil()
{

    ros::NodeHandle nh;
    string prefix_param;
    string service_prefix;
    string service_name;
    //if (nh.searchParam("/kuka_msgs_node/prefix", prefix_param))
    //{
    //    nh.getParam(prefix_param, service_prefix);
    //    service_prefix = service_prefix + "/";
    //}
    //else
    //    service_prefix = "/";


    //service_name = service_prefix + "get_control_type";
    service_name = "get_control_type";
    client_get_control_type_ = nh.serviceClient<kuka_msgs::GetControlType>(service_name.c_str());
    ROS_INFO("Setup service kuka_msgs -> %s", service_name.c_str());


    service_name = service_prefix + "request_control_type";
    client_req_control_type_ = nh.serviceClient<kuka_msgs::RequestControlType>(service_name.c_str());
    ROS_INFO("Setup service kuka_msgs -> %s", service_name.c_str());


}


KukaMsgsUtil::~KukaMsgsUtil()
{

}


kuka_msgs::ControlType KukaMsgsUtil::getControlType()
{
    kuka_msgs::GetControlType::Request request;
    kuka_msgs::GetControlType::Response response;

    if (!client_get_control_type_.exists())
    {
        ROS_WARN_STREAM("Service get_control_type does not exist!");
    }
    client_get_control_type_.call(request, response);

    return(response.control_type);
}

industrial_msgs::ServiceReturnCode KukaMsgsUtil::requestControlType(kuka_msgs::ControlType requestedType)
{
    kuka_msgs::RequestControlType::Request request;
    kuka_msgs::RequestControlType::Response response;

    if (!client_req_control_type_.exists())
    {
        ROS_WARN_STREAM("Service req_control_type does not exist!");
    }
    request.request_type.control_type = requestedType.control_type;
    client_req_control_type_.call(request, response);

    return(response.code);
}



} // namespace kuka_msgs
