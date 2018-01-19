#pragma once
#include "xmlreader.h"
#include <string>
namespace gmml
{
	class Geo3DProjectMetaData;
	class Geo3DMapMetaData;
	class Geo3DMap;
	class Geo3DProject;
}
class GMXMLFILEIO_EXPORT GeoModelXMLReader :
	public XMLReader
{
public:
	GeoModelXMLReader(void);
	~GeoModelXMLReader(void);

	gml::AbstractGML* GetGeomodel();

	///@brief  读取GeoModel
	///@param  DocName 包含待读取GeoModel的文件
	///@return 如果读取成功，返回true；否则返回false
	///@author zhaowei
	///@date   2013.11.04
	
     bool ReadNode(xmlNodePtr curNode);

protected:

	///@brief  创建新的Geo3DProject对象
	///@param 
	///@return 
	///@author zhaowei
	///@date   2013.11.04
	void CreateGeomodel();
};


