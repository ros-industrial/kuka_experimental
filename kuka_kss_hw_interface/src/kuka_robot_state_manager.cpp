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





#include <kuka_kss_hw_interface/kuka_robot_state_manager.h>



namespace kuka_kss_hw_interface
{

KukaRobotStateManager::KukaRobotStateManager() :
    robots_(), next_ID_(0)
{
}

KukaRobotStateManager::~KukaRobotStateManager()
{
}


KukaRobotStateManager* KukaRobotStateManager::getManager()
{
    static KukaRobotStateManager manager_;
    return(&manager_);
}



int KukaRobotStateManager::addRobot()
{
    auto mgr = KukaRobotStateManager::getManager();
    int id = mgr->next_ID_;
    mgr->next_ID_ += 1;
    mgr->robots_.resize(mgr->next_ID_);
    mgr->robots_[id] = new KukaRobotState();
    return(id);
}


KukaRobotState& KukaRobotStateManager::getRobot(int ID)
{
    auto mgr = KukaRobotStateManager::getManager();

    if (ID >= mgr->robots_.size())
        throw std::runtime_error("Invalid robot ID.");

    return( *(mgr->robots_[ID]) );
}



} //namespace kuka_kss_hw_interface
