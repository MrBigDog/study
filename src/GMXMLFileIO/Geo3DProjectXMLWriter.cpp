//#include "StdAfx.h"
#include "Geo3DProjectXMLWriter.h"
#include "Geo3DMapXMLWriter.h"
#include "GeoModelXMLWriter.h"
#include "Geo3DProject.h"
#include "Geo3DMap.h"
#include "GeoModel.h"
#include "GeometryXMLWriter.h"
#include <fstream>
#include "iconv.h"
#include <set>
#include "vtkWin32ProcessOutputWindow.h"

#include "iconv.h"

Geo3DProjectXMLWriter::Geo3DProjectXMLWriter(void)
{
}


Geo3DProjectXMLWriter::~Geo3DProjectXMLWriter(void)
{
}

bool Geo3DProjectXMLWriter::WriteProject(gmml::Geo3DProject* project, const char* xml_path, char* pEncoding, int binary) const
{
	GeometryXMLWriter::BinaryExport = binary;

	if (!project)
		return false;

	std::ofstream xml_file(xml_path);
	if (!xml_file.is_open())
		return false;

	std::string str_dir, strXmlPath(xml_path);
	size_t pos = strXmlPath.rfind('\\');
	if (pos != std::string::npos)
		str_dir = strXmlPath.substr(0, pos + 1);

	char pDataBuf[100] = { 0 };
	sprintf(pDataBuf, "<?xml version=\"1.0\" encoding=\"%s\"?>", pEncoding);
	xml_file << pDataBuf << std::endl

		//xml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl
		<< "<Geo3DProject" << " xmlns:geo3dml=\"" << xmlNS_GMML << "\" xmlns=\"" << xmlNS_GMML << "\"" << std::endl
		<< "\t" << "xmlns:gml=\"" << xmlNS_gml << "\"" << std::endl
		<< "\t" << "xmlns:dt=\"" << xmlNS_gml << "\"" << std::endl
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
		<< "\t" << "xsi:schemaLocation=\"" << xmlNS_GMML << " file:/E:/GMML/docs/schema/gmml.xsd\">" << std::endl;

	xml_file << "<Name>" << project->getName() << "</Name>" << std::endl;
	xml_file << "<Description>" << project->getDescription() << "</Description>" << std::endl;

	// GeoMaps
	xml_file << "<Maps>" << std::endl;
	int map_count = project->GetGeo3DMapCount();
	for (int i = 0; i < map_count; i++)
	{
		gmml::Geo3DMap* map = project->GetGeo3DMap(i);
		if (map)
		{
			std::string map_path = str_dir + map->getName() + ".xml";	//绝对路径
			xml_file << "<Map>" << std::endl;
			Geo3DMapXMLWriter map_writer;
			map_writer.WriteMap(map, map_path, pEncoding);

			map_path = map->getName() + ".xml"; // 相对路径
			xml_file << "<xi:include href=\"" << map_path << "\" />" << std::endl;
			xml_file << "</Map>" << std::endl;
		}
	}
	xml_file << "</Maps>" << std::endl;

	// GeoModels
	xml_file << "<Models>" << std::endl;
	int model_count = project->GetGeoModelCount();
	for (int i = 0; i < model_count; i++)
	{
		gmml::GeoModel* model = project->GetGeoModel(i);
		if (model)
		{
			xml_file << "<Model>" << std::endl;
			std::string model_path = str_dir + model->getName() + "_mod.xml";	//绝对路径

			GeoModelXMLWriter model_writer;
			model_writer.WriteModel(model, model_path, pEncoding);

			model_path = model->getName() + "_mod.xml"; // 相对路径
			xml_file << "<xi:include href=\"" << model_path << "\" />" << std::endl;
			xml_file << "</Model>" << std::endl;
		}
	}
	xml_file << "</Models>" << std::endl;
	xml_file << "<Style>" << std::endl;
	xml_file << "<GeoSceneStyle>" << std::endl;
	xml_file << "<Background>" << "0 0 0";	xml_file << "</Background>" << std::endl;
	xml_file << "<Light>" << std::endl;
	xml_file << "<On>" << "1";	xml_file << "</On>" << std::endl;
	xml_file << "<Type>" << "Scene";	xml_file << "</Type>" << std::endl;
	xml_file << "<Position>" << "0 0 0";	xml_file << "</Position>" << std::endl;
	xml_file << "<FocalPosition>" << "0 0 0";	xml_file << "</FocalPosition>" << std::endl;
	xml_file << "<Intensity>" << "1";	xml_file << "</Intensity>" << std::endl;
	xml_file << "<AmbientColor>" << "0 0 0";	xml_file << "</AmbientColor>" << std::endl;
	xml_file << "<SpecularColor>" << "0 0 0";	xml_file << "</SpecularColor>" << std::endl;
	xml_file << "<DiffuseColor>" << "0 0 0";	xml_file << "</DiffuseColor>" << std::endl;
	xml_file << "</Light>" << std::endl;
	xml_file << "</GeoSceneStyle>" << std::endl;
	xml_file << "</Style>" << std::endl;
	xml_file << "</Geo3DProject>" << std::endl;

	return true;
}
