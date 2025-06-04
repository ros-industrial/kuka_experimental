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


#include <kuka_kss_hw_interface/kuka_commands.h>


namespace kuka_kss_hw_interface
{

// Private constructor for the singleton pattern
kuka_commands::kuka_commands() {
    // Initialize the map with command strings and CommandInfo objects
    long long initTime = 0;
    commands_["Connect-"] = { ROSC_Init, initTime };
    commands_["CmdJpos-"] = { ROSC_CmdJpos, initTime };
    commands_["Start---"] = { ROSC_Start, initTime };
    commands_["Stop----"] = { ROSC_Stop, initTime };
    commands_["Reset---"] = { ROSC_Reset, initTime };
    commands_["DriveOn-"] = { ROSC_DrivesOn, initTime };
    commands_["DriveOff"] = { ROSC_DrivesOff, initTime };
    commands_["Change--"] = { ROSC_Change, initTime };
    commands_["HB------"] = { ROSC_Heartbeat, initTime };
    commands_["Init----"] = { ROSC_SInit, initTime };
    commands_["InitName"] = { ROSC_SInitName, initTime };
    commands_["InitMod-"] = { ROSC_SInitMod, initTime };
    commands_["InitRobV"] = { ROSC_SInitRobV, initTime };
    commands_["JState--"] = { ROSC_SjState, initTime };
    commands_["Status--"] = { ROSC_Sstatus, initTime };
    commands_["Ack-----"] = { ROSC_Sackn, initTime };

    nullCmd_ = { ROSC_Null, 0 };
    nullCmdStr_ = "Null----";
    nullCmdPair_[nullCmdStr_] = nullCmd_;
}


// Static method to get the singleton instance
kuka_commands& kuka_commands::getInstance()
{
    static kuka_commands instance;  // Guaranteed to be created only once
    return instance;
}

// Method to get the CommandInfo (ID and timestamp) by string
kuka_commands::CommandInfo& kuka_commands::getCommandInfo(const std::string& cmd)
{
    try {
        return(commands_.at(cmd));
    } catch (std::out_of_range ex)
    {
        return nullCmd_;  // Return default CommandInfo if not found
    }
}


kuka_commands::CommandInfo& kuka_commands::getCommandInfo(CommandType ID)
{
    auto c = findCommmandID(ID);
    return(c->second);
}

std::map<std::string, kuka_commands::CommandInfo>::iterator kuka_commands::findCommmandID(CommandType ID)
{
    for (auto pair = commands_.begin(); pair != commands_.end(); ++pair)
    {
        if (pair->second.commandID == ID) {
            return pair;
        }
    }
    return (nullCmdPair_.begin());  // Return error pair if ID not found
}


// Method to get the command string by integer ID
std::string kuka_commands::getCommand(CommandType ID)
{
    auto c = findCommmandID(ID);
    std::string sID = c->first;
    return(sID);
}



void kuka_commands::setTimestamp(CommandType ID, long long t)
{
    auto c = findCommmandID(ID);
    c->second.timeStamp = t;
}


long long kuka_commands::getTimestamp(CommandType ID)
{
    auto c = findCommmandID(ID);
    return(c->second.timeStamp);
}


} // namespace kuka_kss_hw_interface


