#pragma once
#include "xmlreader.h"
namespace gmml
{
	class Geo3DModel;
	class GeologicFeatureClass;
	class Geo3DMap;
	class Geo3DProject;
}
class GeologicFeatureXMLReader :
	public XMLReader
{
public:
	GeologicFeatureXMLReader(void);
	~GeologicFeatureXMLReader(void);
	gml::AbstractGML* GetGeologicFeature();

	///@brief 读取GeologicFeature
	///@curNode为当前GeologicFeature节点
	///@return 如果成功返回true 否则返回false
	///@author yang hongjun
	///@date 2013.11.12
	virtual bool ReadGeologicFeature(xmlNode* geologicFeatureNode);
protected:
	///@brief  创建新的GeologicFeature对象	
	///@param 
	///@return 
	///@author yang hongjun
	///@date   2013.11.12
	void CreateGeologicFeature();
	gml::AbstractGML *GeologicFeature_;
};
