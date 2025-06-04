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



#include <iostream>
#include <string>
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>
#include <rapidxml/rapidxml_print.hpp>
#include <kuka_kss_hw_interface/XmlTemplate.h>


using namespace std;
using namespace rapidxml;

//namespace local {

XmlTemplate::XmlTemplate()
{

}

XmlTemplate::XmlTemplate(const char* fileName)
{
	std::ifstream inFile(fileName, std::ios::in);
	if(!inFile.is_open()) {
	    // throw exception...
		std::string errStr = "XmlTemplate could not open file:";
		errStr += fileName;
		throw invalid_argument(errStr.c_str());
	}

	xmlInputStr.assign(std::istreambuf_iterator<char>(inFile), std::istreambuf_iterator<char>());
	inFile.close();
	//cout<< "XmlTemplate read file contents:" <<xmlInputStr << "\n";
	//cout << "parsing xml...\n";
	try {
		char * inStr = (char*)xmlInputStr.c_str();
		xmlDoc.parse<0>(inStr);
	} catch (parse_error& err) {
		cout << "Exception parsing file:" << fileName << endl;
		cout << err.what() << endl;
		//char *errPos = (char*)err.where();
		//cout << "  at :"<< errPos << endl;
		throw err;
	}
	//cout << "done parsing.\n";
}


XmlTemplate::~XmlTemplate()
{
}

void XmlTemplate::setXmlTemplate(string xmlStr)
{
    xmlInputStr.assign(xmlStr);
	xmlDoc.parse<0>((char*)xmlInputStr.c_str());
}

/**
 * Return an XML string of the content.
 *
 */
string XmlTemplate::asXml()
{
	xmlOutputStr.clear();
	rapidxml::print(std::back_inserter(xmlOutputStr), xmlDoc, print_no_indenting);

	return(xmlOutputStr);
}



//} /* namespace local */
