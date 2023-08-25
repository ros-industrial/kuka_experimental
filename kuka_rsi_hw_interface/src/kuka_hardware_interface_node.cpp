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

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;

  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open())
  {
    realtime_file >> has_realtime;
  }
  if (has_realtime)
  {
    const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
    if (max_thread_priority != -1)
    {
      // We'll operate on the currently running thread.
      pthread_t this_thread = pthread_self();

      // struct sched_param is used to store the scheduling priority
      struct sched_param params;

      // We'll set the priority to the maximum.
      params.sched_priority = max_thread_priority;

      int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
      if (ret != 0)
      {
        ROS_ERROR_STREAM("Unsuccessful in setting main thread realtime priority. Error code: " << ret);
      }
      // Now verify the change in thread priority
      int policy = 0;
      ret = pthread_getschedparam(this_thread, &policy, &params);
      if (ret != 0)
      {
        ROS_ERROR_STREAM("Couldn't retrieve real-time scheduling parameters");
      }

      // Check the correct policy was applied
      if (policy != SCHED_FIFO)
      {
        ROS_ERROR("Main thread: Scheduling is NOT SCHED_FIFO!");
      }
      else
      {
        ROS_INFO("Main thread: SCHED_FIFO OK");
      }

      // Print thread scheduling priority
      ROS_INFO_STREAM("Main thread priority is " << params.sched_priority);
    }
    else
    {
      ROS_ERROR("Could not get maximum thread priority for main thread");
    }
  }

  kuka_rsi_hw_interface::KukaHardwareInterface kuka_rsi_hw_interface;
  kuka_rsi_hw_interface.configure();

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  controller_manager::ControllerManager controller_manager(&kuka_rsi_hw_interface, nh);

  kuka_rsi_hw_interface.start();

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  // Run as fast as possible
  while (ros::ok())
  //while (!g_quit)
  {
    // Receive current state from robot
    if (!kuka_rsi_hw_interface.read(timestamp, period))
    {
      ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to read state from robot. Shutting down!");
      ros::shutdown();
    }

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update the controllers
    controller_manager.update(timestamp, period);

    // Send new setpoint to robot
    kuka_rsi_hw_interface.write(timestamp, period);
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  return 0;

}
