#pragma once
#include "xmlwriter.h"

namespace gmml
{
	class GeologicFeatureClass;
	class GeologicFeature;
}

class GMXMLFILEIO_EXPORT FeatureClassXMLWriter : public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.18
	FeatureClassXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.03.18
	~FeatureClassXMLWriter(void);

	///@brief 将FeatureClass写入文件
	///@param feature_class 指向FeatureClass的指针
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.18
	void WriteFeatureClass(gmml::GeologicFeatureClass* feature_class, std::ofstream& xml_stream);

	///@brief 将FeatureClass中的Schema写入文件
	///@param feature_class 指向FeatureClass的指针
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.19
	void WriteSchema(gmml::GeologicFeatureClass* feature_class, std::ofstream& xml_stream);

	///@brief 将FeatureClass中的Feature写入文件
	///@param feature_class 指向FeatureClass的指针
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.19
	void WriteMember(gmml::GeologicFeatureClass* feature_class, std::ofstream& xml_stream);
};

