#pragma once
#include "xmlwriter.h"
namespace gmml
{
	class GeoModel;	
}

class GMXMLFILEIO_EXPORT GeoModelXMLWriter : public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.18
	GeoModelXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.03.18
	~GeoModelXMLWriter(void);

	///@brief 将Model数据写入文件
	///@param model 指向Model数据的指针
	///@param xml_path Model文件的路径
	///@author zhaowei
	///@date 2014.03.18
	bool WriteModel(gmml::GeoModel* model, const std::string& xml_path,char* pEncoding = "gb2312");
};

