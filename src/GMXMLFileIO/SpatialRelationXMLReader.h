#pragma once
#include "XMLReader.h"
class GMXMLFILEIO_EXPORT SpatialRelationXMLReader :
	public XMLReader
{
public:
	SpatialRelationXMLReader(void);
	~SpatialRelationXMLReader(void);

	gml::AbstractGML* GetSpatialRelation();

	///@brief 读取SpatialRelation,featurerelationNode为当前SpatialRelation节点
	///@return 如果成功返回true 否则返回false
	///@author yang hongjun
	///@date 2013.11.13
	virtual bool ReadSpatialRelation(xmlNode* spatialrelationNode);
protected:
	///@brief  创建新的SpatialRelation对象
	///@author yang hongjun
	///@date   2013.11.13
	void CreateSpatialRelation();
	gml::AbstractGML *SpatialRelation_;
};
