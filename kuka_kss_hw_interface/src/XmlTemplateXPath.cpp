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
#include <vector>
#include <kuka_kss_hw_interface/XmlTemplateXpath.h>

using namespace std;
using namespace rapidxml;

//
XmlTemplateXpath::XmlTemplateXpath()
{
	xPathSep = "@/";
	defaultStrBufSize = 64;
}

XmlTemplateXpath::XmlTemplateXpath(const char* fileName) : XmlTemplate(fileName)
{
	xPathSep = "@/";
	defaultStrBufSize = 64;
}

XmlTemplateXpath::~XmlTemplateXpath() {
}

void XmlTemplateXpath::setXmlTemplate(string xmlStr)
{
    xPathAccessMap.clear();
    XmlTemplate::setXmlTemplate(xmlStr);
}

/**
 * Finds the xml nodes in the doc based on a simple XPath lookup string.
 * The strings may include tag names indicated by '/'
 * or may include a trailing attribute reference at the end indicated by '@'.
 * Sample xPathStr "/tagL1/tagL2"
 *                 "/tagL1/tagL2@attr1"
 *                 "/tagL1@attr2"
 * This also keeps a map of previously found paths for quick reuse.
 */
xml_base<>* XmlTemplateXpath::findVal(string xPathStr)
{
	xml_base<> *base;

	// Check map for previous access.
	if (xPathAccessMap.count(xPathStr) > 0)
	{
		//cout << "findVal map lookup found: " << xPathStr <<" \n ";
		base = xPathAccessMap[xPathStr];
	}
	else
	{
		//cout << "findVal starting path parse. \n ";
		string curName;
		xml_node<>* curNode = (xml_node<>*)&xmlDoc;
		xml_attribute<>* curAttr;
		// first char should always be '/'
		size_t lastPos = 0;
		size_t charPos = xPathStr.find_first_of(xPathSep, lastPos+1);
		char lastSep = xPathStr[lastPos];  // Keep track of which separator type ('/'=node, '@'=attribute)
		// xPathStr should always start with '/'
		if (lastSep != '/')
		{
			throw invalid_argument("XmlTemplateXpath path must start with /");
		}
		bool done = false;  // use done flag since
		while(!done)
		{
			//cout << "  found separator at idx = " << charPos << " sep=" << lastSep << "\n";
			// Last name segment will not find a separator
			if (charPos == std::string::npos) done = true;
			if (done)
				// last name segment goes to end of string.
				curName = xPathStr.substr(lastPos+1);
			else
				curName = xPathStr.substr(lastPos+1, (charPos-lastPos-1));

			switch ( lastSep )
			{
			case '/':
				//cout << "  xpath has name = " << curName << "\n";
				curNode = curNode->first_node(curName.c_str());
				if (curNode == NULL)
				{
					string errStr = "XmlTemplateXpath could not find element " + curName;
					throw invalid_argument(errStr.c_str());
				}
				break;
			case '@':
				//cout << "  xpath has attr = " << curName << "\n";
				curAttr = curNode->first_attribute(curName.c_str());
				// attribute should always be end of the string.
				if (curAttr == NULL)
				{
					string errStr = "XmlTemplateXpath could not find attribute " + curName;
					throw invalid_argument(errStr.c_str());
				}
				break;
			default:
				//error
				break;
			}
			if (!done)
			{
				lastSep = xPathStr[charPos];
				lastPos = charPos;
				charPos = xPathStr.find_first_of(xPathSep, lastPos+1);
			}
		}

		if (lastSep == '/')
		{
			// Get a data node if it exists...
// Fix problem when searching for base node of doc....
//			xml_node<>* dataNode = curNode->first_node();
//			if (dataNode != NULL)
//				base = dataNode;
//			else
//				base = curNode;
            base = curNode;
        }
		else if (lastSep == '@')
		{
			base = curAttr;
		}

		// Allocate a buffer big enough for setting data
		size_t slen = base->value_size();
		char *newStr = xmlDoc.allocate_string(base->value(), defaultStrBufSize);
		base->value(newStr, slen);

		// Store the results in the map...
		xPathAccessMap[xPathStr] = base;
	}
	return (base);
}




xml_node<>* XmlTemplateXpath::getNode(string xPathStr)
{
	// Make sure xPath does not contain an attribute
	size_t charPos = xPathStr.find_first_of("@");
	if (charPos != std::string::npos)
	{
		throw invalid_argument("XmlTemplateXpath getNode xPathStr must not reference an attribute.");
	}

	xml_base<>* dataNode = findVal(xPathStr);
	xml_node<>* elNode = static_cast<xml_node<>*>(dataNode);
	// dataNode will contain the actual data if possible, but we want the element node.
	if (elNode->type() != rapidxml::node_element)
		elNode = elNode->parent();

	return(elNode);
}

xml_attribute<>* XmlTemplateXpath::getAttr(string xPathStr)
{
	// Make sure xPath contains an attribute
	size_t charPos = xPathStr.find_first_of("@");
	if (charPos == std::string::npos)
	{
		throw invalid_argument("XmlTemplateXpath getAttr xPathStr must reference an attribute.");
	}

	xml_base<>* dataNode = findVal(xPathStr);
	xml_attribute<>* attrNode = static_cast<xml_attribute<>*>(dataNode);

	return(attrNode);
}


char* XmlTemplateXpath::get(string xPathStr)
{
	xml_base<>* dataNode = findVal(xPathStr);
	return(dataNode->value());
}

char* XmlTemplateXpath::get(const char* xPathStr)
{
	string xp(xPathStr);
	return(get(xp));
}


void XmlTemplateXpath::set(string xPathStr, char* value)
{
	xml_base<>* dataNode = findVal(xPathStr);
	dataNode->value(value);
}

void XmlTemplateXpath::set(const char* xPathStr, char* value)
{
	string xp(xPathStr);
	set(xp, value);
}


void XmlTemplateXpath::set(string xPathStr, char* value, std::size_t size)
{
	xml_base<>* dataNode = findVal(xPathStr);
	dataNode->value(value, size);
}


void XmlTemplateXpath::set(const char* xPathStr, char* value, std::size_t size)
{
	string xp(xPathStr);
	set(xp, value, size);
}

//}/* namespace local */
