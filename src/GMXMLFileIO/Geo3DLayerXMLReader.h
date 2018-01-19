#pragma once
#include "xmlreader.h"
namespace gmml
{
	class Geo3DProjectMetaData;
	class Geo3DMapMetaData;
	class Geo3DMap;
	class Geo3DProject;
}
class GMXMLFILEIO_EXPORT Geo3DLayerXMLReader :
	public XMLReader
{
public:
	Geo3DLayerXMLReader(void);
	~Geo3DLayerXMLReader(void);

	gml::AbstractGML* GetGeoLayer();

	///@brief 读取GeoLayer
	///@curNode为当前GeoLayer节点
	///@return 如果成功返回true 否则返回false
	///@author yang hongjun
	///@date 2013.11.07
	virtual bool ReadGeoLayer(xmlNode* geolayerNode);
protected:
	///@brief  创建新的Geo3DLayer对象
	///@param  
	///@return 
	///@author yang hongjun
	///@date   2013.11.07
	void CreateGeoLayer();
	gml::AbstractGML *GeoLayer_;
};