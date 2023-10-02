/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Kuka Robotics Corp
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


#ifndef XMLTEMPLATE_H_
#define XMLTEMPLATE_H_

#include <string>
#include <rapidxml/rapidxml.hpp>

using namespace std;
using namespace rapidxml;

//namespace local {


/*
 * This class has basic capability to load xml and output it as a string.
 * It is intended as a base class for others that will manipulate data and output it.
 */
class XmlTemplate {
public:
	XmlTemplate();
	XmlTemplate(const char* fileName);
	virtual ~XmlTemplate();

	void setXmlTemplate(string xmlStr);
	string asXml();

protected:
	string xmlInputStr;
	xml_document<> xmlDoc;

	string xmlOutputStr;
};

//} /* namespace local */
#endif /* XMLTEMPLATE_H_ */
