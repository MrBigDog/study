#include "Geo3DMapXMLWriter.h"
#include "Geo3DStyleXMLWriter.h"
#include "Geo3DMap.h"
#include "Geo3DMapMetaData.h"
#include "Geo3DLayer.h"
#include "FeatureClass.h"
#include "Geo3DStyle.h"


#include <fstream>
#include "iconv.h"
#include <string>
#include "guiProgressWindow.h"

extern guiProgressWindow* g_ProgressWindow;

//class Geo3DStyleXMLWriter;

namespace gmml
{
	class Geo3DMap;
	class Geo3DMapMetaData;
	class Geo3DLayer;
	class GeologicFeatureClass;
	class Geo3DStyle;
}

Geo3DMapXMLWriter::Geo3DMapXMLWriter(void)
{
}


Geo3DMapXMLWriter::~Geo3DMapXMLWriter(void)
{
}

bool Geo3DMapXMLWriter::WriteMap(gmml::Geo3DMap* map,const std::string& xml_path,char* pEncoding  )
{
	if(!map)
		return false;

	std::ofstream xml_file(xml_path.c_str());
	if (!xml_file.is_open())
		return false;

	// ÎÄ¼þÍ·
	std::string root_name = "";
	char pDataBuf[100]={0};
	sprintf(pDataBuf,"<?xml version=\"1.0\" encoding=\"%s\"?>",pEncoding);
	xml_file << pDataBuf<< std::endl

		<< "<" << "Geo3DMap" << " xmlns:geo3dml=\"" << xmlNS_GMML << "\" xmlns=\"" << xmlNS_GMML << "\"" << std::endl
		<< "\t" << "xmlns:gml=\"" << xmlNS_gml << "\"" << std::endl
		<< "\t" << "xmlns:gmlcov=\"" << xmlNS_gml << "\"" << std::endl
		<< "\t" << "xmlns:swe=\"" << xmlNS_swe << "\"" << std::endl
		<< "\t" << "xmlns:ogc=\"" << xmlNS_ogc << "\"" << std::endl
		<< "\t" << "xmlns:se=\"" << xmlNS_se << "\"" << std::endl
		<< "\t" << "xmlns:gsml=\"" << xmlNS_gsml << "\"" << std::endl
		<< "\t" << "xmlns:gsmlst=\"" << xmlNS_gsmlst << "\"" << std::endl
		<< "\t" << "xmlns:gsmlgu=\"" << xmlNS_gsmlgu << "\"" << std::endl
		<< "\t" << "xmlns:xlink=\"" << xmlNS_xlink << "\"" << std::endl
		<< "\t" << "xmlns:xsi=\"" << xmlNS_xsi << "\"" << std::endl
		<< "\t" << "xmlns:xi=\"http://www.w3.org/2001/XInclude\"" << std::endl
		<< "\t" << "xmlns:GMML=\"http://www.cgs.gov.cn/GeoGML\"" << std::endl
		<< "\t" << "xsi:schemaLocation=\"" << xmlNS_GMML << " file:/E:/GMML/docs/schema/gmml.xsd\">" << std::endl;

	xml_file << "<Name>" << map->getName() << "</Name>" << std::endl;
	xml_file << "<Description>" << map->getName() << "</Description>" << std::endl;

	xml_file << "<Layers>" <<std::endl;
	int layer_count = map->GetGeo3DLayerCount();
	for(int i = 0; i < layer_count; i++)
	{
		xml_file << "<Layer>" <<std::endl;

		gmml::Geo3DLayer* layer = map->GetLayer(i);
		if(layer)
		{
			xml_file << "<gml:name>" << layer->getName();
			xml_file << "</gml:name>" <<std::endl;
			if(layer->GetFeatureClass())
			{
				xml_file << "<FeatureClass xlink:type=\"simple\" xlink:href=\"#" << layer->GetFeatureClass()->getID() << "\" />" << std::endl;
			}
			
			xml_file << "<Styles>" <<std::endl;
			
			Geo3DStyleXMLWriter style_writer;
			if(layer->GetStyle())
			{
				xml_file << "<Style>" <<std::endl;
				style_writer.WriteStyle(layer->GetStyle(), xml_file);
				xml_file << "</Style>" <<std::endl;
			}
			
			xml_file << "</Styles>" <<std::endl;
		}
		xml_file << "</Layer>" <<std::endl;
	}
	xml_file << "</Layers>" << std::endl;
	xml_file << "</" << "Geo3DMap" << ">" << std::endl;
	return true;
}
