#ifndef KUKA_COMMANDS_H
#define KUKA_COMMANDS_H

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

#include <iostream>
#include <string>
#include <map>
#include <ctime>  // For time functionality


namespace kuka_kss_hw_interface
{

// Singleton class to manage communication message strings & IDs.
// Also has capability to mark last command timestamp for each command.
class kuka_commands {
private:
    // Private constructor for the singleton pattern
    kuka_commands();
    // Disable copy constructor and assignment operator
    kuka_commands(const kuka_commands&) = delete;
    kuka_commands& operator=(const kuka_commands&) = delete;


public:
    // Structure to hold command information (ID and timeStamp)
    // Command ID constants
    enum CommandType
    {
        ROSC_Init=0,
        ROSC_CmdJpos=1,
        ROSC_Start=2,
        ROSC_Stop=3,
        ROSC_Reset=4,
        ROSC_DrivesOn=5,
        ROSC_DrivesOff=6,
        ROSC_Change=7,
        ROSC_Heartbeat=8,
        ROSC_SInit=100,
        ROSC_SInitName=101,
        ROSC_SInitMod=102,
        ROSC_SInitRobV=103,
        ROSC_SjState=104,
        ROSC_Sstatus=105,
        ROSC_Sackn=106,
        ROSC_Null=-1 // Error code - do not use
    };


    struct CommandInfo {
        CommandType commandID;
        long long timeStamp;
    };


    // Static method to get the singleton instance
    static kuka_commands& getInstance();

    // Method to get the CommandInfo (ID and timestamp) by string
    CommandInfo& getCommandInfo(const std::string& cmd) ;

    // Method to get the CommandInfo (ID and timestamp) by ID
    CommandInfo& getCommandInfo(CommandType ID) ;


    // Method to get the command string by integer ID
    std::string getCommand(CommandType ID);

    void setTimestamp(CommandType ID, long long t);
    long long getTimestamp(CommandType ID);

    // Overload operator[] to return CommandInfo (alias for getCommandInfo)
    CommandInfo& operator[](const std::string& cmd)  {
        return getCommandInfo(cmd);
    }

    // Overload operator[] to return command string (alias for getCommand)
    std::string operator[](CommandType ID) {
        return getCommand(ID);
    }

private:
    std::map<std::string, CommandInfo>::iterator findCommmandID(CommandType ID);

    // Map that holds command string to CommandInfo mappings
    std::map<std::string, CommandInfo> commands_;
    std::map<std::string, CommandInfo> nullCmdPair_;
    CommandInfo nullCmd_;
    std::string nullCmdStr_;

};

} // namespace kuka_kss_hw_interface
#endif // KUKA_COMMAND_H
