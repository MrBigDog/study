#pragma once
#include "xmlreader.h"

class GMXMLFILEIO_EXPORT FeatureClassXMLReader :
	public XMLReader
{
public:
	FeatureClassXMLReader(void);
	~FeatureClassXMLReader(void);

 gml::AbstractGML* GetFeatureClass();
 ///@brief 读取FeatureClass
 ///@featureclassNode为当前FeatureClass节点
 ///@return 如果成功返回true 否则返回false
 ///@author yang hongjun
 ///@date 2013.11.10
virtual bool ReadFeatureClass(xmlNode* featureclassNode);

protected:
	bool ReadField(xmlNode* featureclassNode);
 
	///@brief  创建新的FeatureClass对象
	///@param  
	///@return 
	///@author yang hongjun
	///@date   2013.11.10
	 void CreateFeatureClass();
	 gml::AbstractGML* FeatureClass_;
};

