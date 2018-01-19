#include "GeoStyleXMLReader.h"
#include "Geo3DStyle.h"
#include <libxml/parser.h>//"parser.h"//  [1/5/2018 BigDog]
#include <fstream>
#include "iconv.h"
#include <string>

GeoStyleXMLReader::GeoStyleXMLReader(void)
{
	GeoStyle_=0;
}


GeoStyleXMLReader::~GeoStyleXMLReader(void)
{
	if(GeoStyle_)
		delete GeoStyle_;
}

///@brief 获取GeoStyle对象
///@params 
///@author yang hongjun
///data 2013.11.09
gml::AbstractGML* GeoStyleXMLReader::GetGeoStyle()
{
	return GeoStyle_;
}

///@brief 创建一个新的GeoStyle对象
///@params
///@author yang hongjun
///@data 2013.11.09
void GeoStyleXMLReader::CreateGeoStyle()
{
	
	GeoStyle_=new gmml::Geo3DStyle;
}

///@brief 解析GeoStyle
///@params geolayer引用上层的layer,geostyleNode引用上层的Geo3DLayer中的GeoStyle的节点指针 
        ///解析成功返回true，否则返回false
///@author yang hongjun
///@data 2013.11.09
bool GeoStyleXMLReader::ReadGeoStyle(xmlNode* geostyleNode)
{
	xmlNodePtr curNode=geostyleNode;
	curNode=curNode->xmlChildrenNode;
	xmlNodePtr NodePtr_replica = curNode;
	xmlChar* tmp_str;
	CreateGeoStyle();
	gmml::Geo3DStyle* style=(gmml::Geo3DStyle*)GeoStyle_;

	while(curNode!=NULL)
	{
		if(!xmlStrcmp(curNode->name,(const xmlChar *)"name"))///@brief GeoStyle中包含name节点，
			                                                 ///将其内容取出存储到GeoStyle相应的位置
		{
			    tmp_str=xmlNodeGetContent(curNode);
				char* out=ConvertEnc("utf-8","gb2312", (char*)tmp_str);
				if(out)
					style->setName(out);
				else
				    style->setName((char*)tmp_str);
				     
			
			xmlFree(tmp_str);
		}

		///@brief 读取FeatureTypeStyle
		///@params 因为头文件中没有FeatureTypeStyle的类声明和定义，所以该段代码只是临时性
		///@author yang hongjun
		///@data 2013.11.09
		if(!xmlStrcmp(curNode->name,(const xmlChar *)"FeatureTypeStyle"))
		{
			
			/*FeatureTypeStyleXMLReader FeatureTypeStyleReader;*////@params 临时性的FeatureTypeStyleXMLReader类（未定义未声明）
			/*if(style)
				FeatureTypeStyleReader.ReadFile(style,curNode);
				style->AddFeatureTypeStyle(FeatureTypeStyleReader.GetFeatureTypeStyle());*////@params 临时性的AddFeatureTypeStyle函数（未定义未声明）
		}


		curNode=curNode->next;
	}

	return true;

}