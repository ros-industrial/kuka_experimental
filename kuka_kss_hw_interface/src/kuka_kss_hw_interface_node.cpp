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
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */
#include <exception>
#include <kuka_kss_hw_interface/kuka_kss_hw_interface.h>

void debug_wait(double seconds);



int main(int argc, char** argv)
{
  std::string node_name = "kuka_kss_hw_interface";
  ROS_INFO_STREAM_NAMED(node_name, "Starting hardware interface...");

  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Handle unexpected startup errors
  try{

      //debug_wait(60.0);

      kuka_kss_hw_interface::KukaKssHardwareInterface kuka_kss_hw_interface;
      kuka_kss_hw_interface.configure();

      controller_manager::ControllerManager controller_manager(&kuka_kss_hw_interface, nh);

      kuka_kss_hw_interface.controlLoop(controller_manager);

  }
  catch(std::exception& ex)
  {
      cout << node_name <<": Main run exception:" << endl;
      cout << ex.what() << endl;
      cout << "Exiting." << endl;
      ROS_ERROR_STREAM_NAMED(node_name, ex.what());
  }

  spinner.stop();

  ROS_INFO_STREAM_NAMED(node_name, "Shutting down.");
  ros::shutdown();

  return 0;

}


void debug_wait(double seconds)
{
    ROS_INFO_NAMED("kuka_hardware_interface", "Time delay for debug catch.  Waiting %f sec.", seconds);
    auto t_start = std::chrono::steady_clock::now();
    ros::Duration secTimeout;
    secTimeout.fromSec(seconds);
    ros::Duration secElapsed;
    secElapsed.fromSec(0.0);
    bool doLoop = true;
    while(doLoop)
    {
      auto t_check = std::chrono::steady_clock::now();
      secElapsed.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(t_check - t_start).count());

      if (secElapsed > secTimeout)
      {
          ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Time delay complete.");
          doLoop = false;
      }
    }
}


