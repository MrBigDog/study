#pragma once

#include <string>
#include <vector>

#include "XMLReader.h"
class GMXMLFILEIO_EXPORT Geo3DProjectXMLReader: public XMLReader
{
public:
	///@brief  构造函数
	///@author zhaowei
	///@date   2013.11.04
	Geo3DProjectXMLReader();

	///@brief  析构函数
	///@author zhaowei
	///@date   2013.11.04
	~Geo3DProjectXMLReader();

	///@brief  获取Geo3DProject
	///@param   
	///@return 返回指向Geo3DProject父类的指针
	///@author zhaowei
	///@date   2013.11.04
	gml::AbstractGML* GetGeoProject();

	///@brief  读取Geo3DProject
	///@param projectFile 包含待读取Geo3DProject的文件
	///@param charSet 打开文件的字符集
	///@return 如果读取成功，返回true；否则返回false
	///@author zhaowei
	///@date   2013.11.04
	virtual bool ReadFile(const char *DocName, const std::string charSet,int binary=0);

private:
	///@brief  创建新的Geo3DProject对象
	///@param 
	///@return 
	///@author zhaowei
	///@date   2013.11.04
	void CreateProject();

	gml::AbstractGML* Project_;
};