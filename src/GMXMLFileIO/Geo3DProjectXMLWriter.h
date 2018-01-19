#pragma once

#include <string>
#include <vector>

#include "xmlwriter.h"


namespace gmml
{
	class Geo3DProject;
	class Geo3DProjectMetaData;
	class Geo3DMap;
	class Geo3DMapMetaData;
	class GeoModel;
	class GeoModelMetaData;
}


class GMXMLFILEIO_EXPORT Geo3DProjectXMLWriter : public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.17
	Geo3DProjectXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.03.17
	~Geo3DProjectXMLWriter(void);

	///@brief 向文件中写入Project数据
	///@param project 指向待写入Project的指针
	///@param xml_path 写入文件的路径
	bool WriteProject(gmml::Geo3DProject* project, const char* xml_path,char* pEncoding="gb2312",int binary=0) const;
};

