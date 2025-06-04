/*********************************************************************
 * This software is factored out from the ros_control hardware interface
 * for Universal Robots manipulators. The software assigns the highest
 * realtime priority to the currently running thread.
 * You can find the original work at
 * 
 *     https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/395c054/ur_robot_driver
 *     https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/395c054/ur_robot_driver/src/hardware_interface_node.cpp#L63
 * 
 * Copyright 2019 FZI Forschungszentrum Informatik
 * Created on behalf of Universal Robots A/S
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

/*
 * Author:  Felix Exner <exner@fzi.de>
 */

#ifndef KUKA_RSI_HARDWARE_INTERFACE_ASSIGN_RT_PRIORITY_
#define KUKA_RSI_HARDWARE_INTERFACE_ASSIGN_RT_PRIORITY_

#include <fstream>
#include <ros/ros.h>

namespace kuka_rsi_hw_interface
{
void assign_max_rt_priority()
{
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
}
} // namespace kuka_rsi_hardware_interface

#endif /* KUKA_RSI_HARDWARE_INTERFACE_ASSIGN_RT_PRIORITY_ */