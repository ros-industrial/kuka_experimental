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


#ifndef XMLTEMPLATEXPATH_H_
#define XMLTEMPLATEXPATH_H_

#include <map>

#include <kuka_kss_hw_interface/XmlTemplate.h>

//namespace local {

class XmlTemplateXpath: public XmlTemplate {
public:
	XmlTemplateXpath();
	XmlTemplateXpath(const char* fileName);
	virtual ~XmlTemplateXpath();

    void setXmlTemplate(string xmlStr);
	xml_node<>* getNode(string xPathStr);
	xml_attribute<>* getAttr(string xPathStr);
	char* get(string xPathStr);
	char* get(const char* xPathStr);

	void set(string xPathStr, char *value);
	void set(const char* xPathStr, char *value);

	void set(string xPathStr, char *value, std::size_t size);
	void set(const char* xPathStr, char *value, std::size_t size);

protected:
	std::map<string, xml_base<>*>	xPathAccessMap;
	int 	defaultStrBufSize;

private:
	xml_base<>* findVal(string xPathStr);
	string xPathSep;
};

//} /* namespace local */
#endif /* XMLTEMPLATEXPATH_H_ */
