#pragma once
#include "xmlreader.h"
class GMXMLFILEIO_EXPORT FeatureRelationXMLReader :
	public XMLReader
{
public:
	FeatureRelationXMLReader(void);
	~FeatureRelationXMLReader(void);

 
	///@brief 读取FeatureRelation
	///@featurerelationNode为当前FeatureRelation节点
	///@return 如果成功返回true 否则返回false
	///@author yang hongjun
	///@date 2013.11.13
	virtual bool ReadNode(xmlNode* relationshipNode);

protected:
	bool ReadRelation(xmlNode* relationshipNode);
	///@brief  创建新的FeatureRelation对象
	///@param  
	///@return 
	///@author yang hongjun
	///@date   2013.11.13
	void CreateRelationship(std::string relationType);
	 
};

