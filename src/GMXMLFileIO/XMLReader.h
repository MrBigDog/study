#pragma once
#include "export.h"

#include <libxml/parser.h>
#include <string>
namespace gml
{
	class AbstractGML;
}

class GMXMLFILEIO_EXPORT XMLReader
{
public:
	///@brief  构造函数
	///@author zhaowei
	///@date   2013.11.04
	XMLReader(void);

	///@brief  析构函数
	///@author zhaowei
	///@date   2013.11.04
	~XMLReader(void);
	
	///@brief  读文件
	///@param  docName 文件路径和名称
	///@param  charSet 打开文件时选择的字符集格式
	///@return 如果读取成功，返回true；否则返回false
	///@author zhaowei
	///@date   2013.11.04
	virtual bool ReadFile(const char *docName, const std::string charSet = "UTF-8");

	///@brief  读取XML文件中的指定节点
	///@param  curNode XML文件中标签所在节点的指针
	///@return 如果读取成功，返回true；否则返回false
	///@author zhaowei
	///@date   2013.11.06
	virtual bool ReadNode(xmlNodePtr curNode){return 0;};

	int InitCharSet(const char* pFileName);

    gml::AbstractGML* GetReadObject();

protected:
	///@brief  字符串转换
	///@param  encFrom 被转换编码
	///@param  encTo   转换后的编码
	///@param  in      待转换字符串
	///@return 转换后的字符串
	///@author zhaowei
	///@date   2013.11.04
	char* ConvertEnc(char* encFrom, char* encTo, char* in);

	///@brief  计算形参DocName中左边的空白字符数目
	///@param  DocName 待处理的文件名称
	///@return 返回文件名称左侧的空白字符数
	///@author zhaowei
	///@date   2013.11.04
	int TrimLeft(char* DocName);

	///@brief  计算形参DocName中右边的空白字符数目
	///@param  DocName 待处理的文件名称
	///@return 返回文件名称右侧的空白字符数
	///@author zhaowei
	///@date   2013.11.04
	int TrimRight(char* DocName);

	///@brief  清除形参DocName中的空白字符
	///@param  DocName 待处理的文件名称
	///@return 返回处理后的字符串
	///@author zhaowei
	///@date   2013.11.04
	const char* TrimAll(char* DocName);

 
protected:	
	char FilePath[256];		//文件路径
	char mXMLCoding[256];	//XML编码编号
	char gCharSet[256];

	gml::AbstractGML* gmmlObject;
};