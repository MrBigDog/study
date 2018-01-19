#pragma once
#include "xmlreader.h"
namespace gmml
{
	class Geo3DModel;
	class GeologicFeatureClass;
	class Geo3DMap;
	class Geo3DProject;
}
class GMXMLFILEIO_EXPORT GeoFeatureXMLReader :
	public XMLReader
{
public:
	GeoFeatureXMLReader(void);
	~GeoFeatureXMLReader(void);
	gml::AbstractGML* GetGeoFeature();

	///@brief 读取GeologicFeature
	///@param curNode为当前GeologicFeature节点
	///@return 如果成功返回true 否则返回false
	///@author yang hongjun
	///@date 2013.11.12
	virtual bool ReadNode(xmlNode* geoFeatureNode);
protected:
	// Read geometry Object
	bool ReadMappedFeature(xmlNode* geoNode);

	///@brief  创建新的GeologicFeature对象
	///@param 
	///@return 
	///@author yang hongjun
	///@date   2013.11.12
	void CreateGeoFeature();	
};
