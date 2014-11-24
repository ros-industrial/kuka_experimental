/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
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
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
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
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");

  ros::init(argc, argv, "kuka_rsi_hardware_interface");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  kuka_rsi_hw_interface::KukaHardwareInterface kuka_rsi_hw_interface;
  kuka_rsi_hw_interface.configure();

  // Set up timers
  struct timespec ts = {0, 0};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
  {
    ROS_FATAL("Failed to poll clock!");
  }
  ros::Time last(ts.tv_sec, ts.tv_nsec);
  ros::Time now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  controller_manager::ControllerManager controller_manager(&kuka_rsi_hw_interface, nh);

  kuka_rsi_hw_interface.start();

  // Get current time and elapsed time since last read
  if (!clock_gettime(CLOCK_MONOTONIC, &ts))
  {
    now.sec = ts.tv_sec;
    now.nsec = ts.tv_nsec;
    period = now - last;
    last = now;
  }
  else
  {
    ROS_FATAL("Failed to poll clock!");
  }

  // Run as fast as possible
  while (ros::ok())
  //while (!g_quit)
  {
    // Receive current state from robot
    if (!kuka_rsi_hw_interface.read(now, period))
    {
      ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to read state from robot. Shutting down!");
      ros::shutdown();
    }

    // Get current time and elapsed time since last read
    if (!clock_gettime(CLOCK_MONOTONIC, &ts))
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    }
    else
    {
      ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to poll clock. Shutting Down!");
      ros::shutdown();
    }

    // Update the controllers
    controller_manager.update(now, period);

    // Send new setpoint to robot
    kuka_rsi_hw_interface.write(now, period);
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  return 0;

}
