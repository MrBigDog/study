#pragma once

#include "export.h"

#include <algorithm>
#include <fstream>
#include <string>

#ifndef _IncludeXML_
#define _IncludeXML_
const std::string  xmlNS_GMML = "http://www.cgs.gov.cn/GMML";
const std::string  xmlNS_gml	 = "http://www.opengis.net/gml/3.2";
const std::string  xmlNS_CityGML= "http://www.opengis.net/citygml/1.0";
const std::string  xmlNS_xlink	 = "http://www.w3.org/1999/xlink";
const std::string  xmlNS_xsi	 = "http://www.w3.org/2001/XMLSchema-instance";
const std::string  xmlNS_swe	 = "http://www.opengis.net/swe/2.0";
const std::string  xmlNS_gsml	 = "http://xmlns.geosciml.org/GeoSciML-Core/3.0";
const std::string  xmlNS_gsmlst = "http://xmlns.geosciml.org/GeologicStructure/3.0";
const std::string  xmlNS_gsmlgu = "http://xmlns.geosciml.org/GeologicUnit/3.0";
const  std::string  xmlNS_se	 = "http://www.opengis.net/se";
const std::string  xmlNS_ogc	 = "http://www.opengis.net/ogc";
#endif

class GMXMLFILEIO_EXPORT XMLWriter
{
public:
	XMLWriter(void);
	~XMLWriter(void);
};

