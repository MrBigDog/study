#ifndef XMLMetaDataReader_h__
#define XMLMetaDataReader_h__

#include "XMLReader.h"
#include <libxml/parser.h>//#include "parser.h"//  [1/5/2018 BigDog]

namespace gmml
{
	class Geo3DProjectMetaData;
	class Geo3DMapMetaData;
	class Geo3DMap;
	class Geo3DProject;
}

class XMLMetaDataReader: public XMLReader
{
public:
	///@brief 构造函数
	XMLMetaDataReader(void);

	///@brief 析构函数
	~XMLMetaDataReader(void);

	///@brief  读取GeoProject的元数据
	///@params Geo3DProject* geoproject 该元数据对应的Geo3DProject
	///        xmlNode* metadataNode 该元数据所在的XML结点
	///@return 
	///@author zhaowei
	///@date   2013.11.1
	void ReadGeoProjectMetaData(gmml::Geo3DProject* geoproject, xmlNode* metadataNode);


	///@brief  读取GeoMap的元数据
	///@params Geo3DMap* geomap 该元数据对应的Geo3DMap
	///        xmlNode* metadataNode 该元数据所在的XML结点
	///@return 
	///@author zhaowei
	///@date   2013.11.1
	void ReadGeoMapMetaData(gmml::Geo3DMap* geomap, xmlNode* metadataNode);
};

#endif // XMLMetaDataReader_h__