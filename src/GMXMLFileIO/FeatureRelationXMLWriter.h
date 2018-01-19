#pragma once
#include "xmlwriter.h"

namespace gmml
{
	class FeatureRelation;
	class GeologicFeatureClass;
	class GeoModel;
}

class GMXMLFILEIO_EXPORT FeatureRelationXMLWriter : public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.18
	FeatureRelationXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.03.18
	~FeatureRelationXMLWriter(void);

	///@brief 将FeatureRelation写入文件
	///@param feature_relation 指向FeatureRelation指针
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.18
	void WriteFeatureRelation(gmml::FeatureRelation* feature_relation, std::ofstream& xml_stream);
};

