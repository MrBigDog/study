#pragma once
#include "xmlreader.h"
namespace gmml
{
	class Geo3DProjectMetaData;
	class Geo3DMapMetaData;
	class Geo3DMap;
	class Geo3DProject;
	class Geo3DLayer;
}
class GeoStyleXMLReader :
	public XMLReader
{
public:
	GeoStyleXMLReader(void);
	~GeoStyleXMLReader(void);
	gml::AbstractGML* GetGeoStyle();

	virtual bool ReadGeoStyle(xmlNode* geostyleNode);

protected:
	void CreateGeoStyle();
	gml::AbstractGML* GeoStyle_;
};

