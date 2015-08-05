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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef KUKA_RSI_HW_INTERFACE_RSI_COMMAND_
#define KUKA_RSI_HW_INTERFACE_RSI_COMMAND_

#include <tinyxml.h>
#include <vector>

namespace kuka_rsi_hw_interface
{

class RSICommand
{
public:
  RSICommand();
  RSICommand(std::vector<double> position_corrections, unsigned long long ipoc);
  std::string xml_doc;
};

RSICommand::RSICommand()
{
  // Intentionally empty
}

RSICommand::RSICommand(std::vector<double> joint_position_correction, unsigned long long ipoc)
{
  TiXmlDocument doc;
  TiXmlElement* root = new TiXmlElement("Sen");
  root->SetAttribute("Type", "ImFree");
  TiXmlElement* el = new TiXmlElement("AK");
  // Add string attribute
  el->SetAttribute("A1", std::to_string(joint_position_correction[0]));
  el->SetAttribute("A2", std::to_string(joint_position_correction[1]));
  el->SetAttribute("A3", std::to_string(joint_position_correction[2]));
  el->SetAttribute("A4", std::to_string(joint_position_correction[3]));
  el->SetAttribute("A5", std::to_string(joint_position_correction[4]));
  el->SetAttribute("A6", std::to_string(joint_position_correction[5]));

  root->LinkEndChild(el);
  el = new TiXmlElement("IPOC");
  el->LinkEndChild(new TiXmlText(std::to_string(ipoc)));
  root->LinkEndChild(el);
  doc.LinkEndChild(root);
  TiXmlPrinter printer;
  printer.SetStreamPrinting();
  doc.Accept(&printer);

  xml_doc = printer.Str();
}

} // namespace kuka_rsi_hw_interface

#endif
