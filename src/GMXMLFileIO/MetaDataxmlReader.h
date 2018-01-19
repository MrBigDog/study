#pragma once
#include "XMLReader.h"


namespace gmml
{
	class Geo3DProjectMetaData;
	class Geo3DMapMetaData;
	class GeoModelMetaData;
	class Geo3DMap;
	class Geo3DProject;
	class GeoModel;
}

class GMXMLFILEIO_EXPORT MetaDataXMLReader: public XMLReader
{
public:
	///@brief 构造函数
	MetaDataXMLReader(void);

	///@brief 析构函数
	~MetaDataXMLReader(void);

	///@brief  读取GeoProject的元数据
	///@param Geo3DProject* geoproject 该元数据对应的Geo3DProject
	///        xmlNode* metadataNode 该元数据所在的XML结点
	///@return 
	///@author zhaowei
	///@date   2013.11.1
	void ReadGeoProjectMetaData(gmml::Geo3DProject* geoproject, xmlNode* metadataNode);


	///@brief  读取GeoMap的元数据
	///@param Geo3DMap* geomap 该元数据对应的Geo3DMap
	///        xmlNode* metadataNode 该元数据所在的XML结点
	///@return 
	///@author zhaowei
	///@date   2013.11.1
	void ReadGeoMapMetaData(gmml::Geo3DMap* geomap, xmlNode* metadataNode);


	///@brief  读取GeoModel的元数据
	///@param GeoModel* geomodel 该元数据对应的GeoModel
	///        xmlNode* metadataNode 该元数据所在的XML结点
	///@return 
	///@author yang hongjun
	///@date   2013.11.10
	void ReadGeoModelMetaData(gmml::GeoModel* geomodel, xmlNode* metadataNode);

};