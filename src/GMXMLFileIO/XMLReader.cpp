#include "XMLReader.h"




#include "iconv.h"

#include <fstream>
#include <string>
#include "Geo3DProject.h"
//#include "CsXMLGeoMapReader.h"
//#include "..//gmlFeature//guiProgressWindow.h"

///@brief  构造函数
///@author zhaowei
///@date   2013.11.04
XMLReader::XMLReader(void)
{
 
	strcpy(mXMLCoding,"UTF-8");
	gmmlObject = 0;
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.04
XMLReader::~XMLReader(void)
{
	if(gmmlObject)
	if(gmmlObject->getContainer() == NULL)
		delete gmmlObject;
	gmmlObject = 0;
}

gml::AbstractGML* XMLReader::GetReadObject()
{
	return gmmlObject;
}

///@brief  字符串转换
///@param  encFrom 被转换编码
///        encTo   转换后的编码
///        in      待转换字符串
///@return 转换后的字符串
///@author zhaowei
///@date   2013.11.04
char* XMLReader::ConvertEnc(char* encFrom, char* encTo, char* in)
{
	static char bufin[1024], bufout[1024];
	static char *sin, *sout;
	int mode, lenin, lenout, ret, nline;
	iconv_t c_pt;

	if ((c_pt = iconv_open(encTo, encFrom)) == (iconv_t)-1)
	{
		return NULL;
	}
	iconv(c_pt, NULL, NULL, NULL, NULL);

	lenin  = strlen(in) + 1;
	lenout = 1024;
	sin    = (char*)in;
	sout   = bufout;
	ret = iconv(c_pt,(/*const*/ char**) &sin, (size_t*)&lenin, &sout, (size_t*)&lenout);//  [1/5/2018 BigDog]

	if (ret == -1)
	{
		return NULL;
	}

	iconv_close(c_pt); 
	return bufout;
}

///@brief  计算形参DocName中左边的空白字符数目
///@param  DocName 待处理的文件名称
///@return 返回文件名称左侧的空白字符数
///@author zhaowei
///@date   2013.11.04
int XMLReader::TrimLeft(char* DocName)
{
	int count = 0;
	int len = strlen(DocName);
	for(int i = 0; i < len; i++)
	{
		if(DocName[i] == ' ' || DocName[i] == '\t' || DocName[i] == 10)
		{
			DocName[i]== '\0';
			count++;
		}
		else
			break;
	}
	return count;
}

///@brief  计算形参DocName中右边的空白字符数目
///@param DocName 待处理的文件名称
///@return 返回文件名称右侧的空白字符数
///@author zhaowei
///@date   2013.11.04
int XMLReader::TrimRight(char* DocName)
{
	int count = 0;
	int len = strlen(DocName);
	for(int i = len-1; i >= 0; i--)
	{
		if(DocName[i] == ' ' || DocName[i] == '\t' || DocName[i] == 10)
		{
			DocName[i] == '\0';
			count++;
		}
		else
			break;
	}
	return count;
}

///@brief  清除形参DocName中的空白字符
///@param DocName 待处理的文件名称
///@return 返回处理后的字符串
///@author zhaowei
///@date   2013.11.04
const char* XMLReader::TrimAll(char* DocName)
{
	int leftc = TrimLeft(DocName);
	int rightc = TrimRight(DocName);
	int len = strlen(DocName);
	std::string sre = DocName;
	sre = sre.substr(leftc,len -rightc-leftc);

	return sre.c_str();
}

bool XMLReader::ReadFile(const char *DocName, const std::string charSet)
{
	InitCharSet(DocName);
	xmlDocPtr doc;           //定义解析文档指针
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	xmlChar* tmp_str;        //临时字符串变量

	std::string codeType = charSet;
	if(charSet=="")
	{
		codeType = gCharSet;
	}
	else
	{
		codeType=charSet;
	}
 
	doc = xmlReadFile((char*)DocName, codeType.c_str(), XML_PARSE_RECOVER); //解析文件

//	strcpy(mXMLCoding,codeType.c_str());

	if(doc == NULL) 
	{  
		return false; 
	} 

	curNode = xmlDocGetRootElement(doc); //确定文档根元素
	if(curNode == NULL)
	{ 
		xmlFreeDoc(doc); 
		return false; 
	} 

	bool flag = ReadNode(curNode);
	xmlFreeDoc(doc) ;
	return flag;

}

int XMLReader::InitCharSet(const char* pFileName)
{
	std::string strCharset = "UTF-8";			 
	std::fstream fin;
	fin.open(pFileName,std::ios::in);
	if (fin.is_open())
	{
		std::string strLine;
		getline(fin,strLine);
		if (strLine.find("gb2312")!=std::string::npos||
			strLine.find("GB2312")!=std::string::npos)
		{
			strCharset = "gb2312";
		}
		else if (strLine.find("UTF-8")!=std::string::npos||
			strLine.find("utf-8")!=std::string::npos)
		{
			strCharset = "UTF-8";
		}

		memcpy(gCharSet,strCharset.c_str(),strCharset.length());
		fin.close();
	}

	return 0;
}