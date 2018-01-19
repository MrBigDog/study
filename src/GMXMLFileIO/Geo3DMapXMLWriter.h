#pragma once
#include "xmlwriter.h"
namespace gmml
{
	class Geo3DMap;
}

class GMXMLFILEIO_EXPORT Geo3DMapXMLWriter : public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.17
	Geo3DMapXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.03.17
	~Geo3DMapXMLWriter(void);

	///@brief 向文件中写入Map数据
	///@param map 指向Map的指针
	///@param xml_path Map文件的路径
	///@author zhaowei
	///@date 2014.03.17
	bool WriteMap(gmml::Geo3DMap* map,const std::string& xml_path,char* pEncoding = "gb2312");
};

