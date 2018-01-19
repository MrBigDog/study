#pragma once
#include "xmlreader.h"
namespace gmml
{
	class Geo3DProjectMetaData;
	class Geo3DMapMetaData;
	class Geo3DMap;
	class Geo3DProject;
}
class GMXMLFILEIO_EXPORT Geo3DMapXMLReader :
	public XMLReader
{
public:
	Geo3DMapXMLReader(void);
	~Geo3DMapXMLReader(void);

	gml::AbstractGML* GetGeomap();

	///@brief  读取Geo3DMap
	///@param MapFile 包含待读取Geo3DMap的文件
	///@return 如果读取成功，返回true；否则返回false
	///@author zhaowei
	///@date   2013.11.04
	
	 bool ReadNode(xmlNodePtr curNode);
protected:
	//virtual bool ReadGeo3DMap(const char *DocName);
	///@brief  创建新的Geo3DProject对象
	///@param  
	///@return 
	///@author zhaowei
	///@date   2013.11.04
	void CreateGeomap();
};
	
 

