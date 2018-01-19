#include "stdafx.h"
#include "global.h"

#include "iconv.h"

#include <string.h>
#include <fstream>
#include <string>

char gCharSet[256];

char* ConvertEnc(char* encFrom, char* encTo, char* in)
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

char* ReadValueAndConvert(xmlTextReaderPtr reader)
{
	char* valueout = NULL;

	xmlChar* value2 = xmlTextReaderReadString(reader);
	if (value2)
	{
		//if (!strcmp(gCharSet,"UTF-8")||
		//	!strcmp(gCharSet,"utf-8"))
		//{
			valueout = ConvertEnc("UTF-8","GB2312", (char*)value2);
			return valueout;
		//}
		//else
		//	return (char*)value;

		xmlFree(value2);
	}

	return valueout;
}

char* ReadAttrAndConvert(xmlTextReaderPtr reader,const char* attribute_name)
{
	char* attrout = NULL;
	
	if (xmlTextReaderAttributeCount(reader)>0)
	{
		xmlChar* value1 = xmlTextReaderGetAttribute(reader,(xmlChar*)attribute_name);

		if(value1)
		{
			//if (!strcmp(gCharSet,"UTF-8")||
			//	!strcmp(gCharSet,"utf-8"))
			//{
				attrout = ConvertEnc("UTF-8","GB2312", (char*)value1);
				return attrout;
			//}
			//else
			//	return (char*)value;
		}


		xmlFree(value1);
	}

	return attrout;
}

int TrimLeft(char* DocName)
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

int TrimRight(char* DocName)
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

int InitCharSet(const char* pFileName)
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
	}

	return 0;
}