#include "Geo3DMapXMLReader.h"
#include "Geo3DMap.h"
#include "MetaDataXMLReader.h"
#include "Geo3DLayerXMLReader.h"

#include <fstream>
#include "iconv.h"
#include <string>
#include "gmmlSystem.h"
Geo3DMapXMLReader::Geo3DMapXMLReader(void)
{

}


Geo3DMapXMLReader::~Geo3DMapXMLReader(void)
{

}


///@brief  获取Geo3DMap
///@param 
///@return 返回指向Geo3DMap父类的指针
///@author zhaowei
///@date   2013.11.04
gml::AbstractGML* Geo3DMapXMLReader::GetGeomap()
{
	return gmmlObject;
}



///@brief  创建新的Geo3DMap对象
///@param 
///@return 
///@author zhaowei
///@date   2013.11.04
void Geo3DMapXMLReader::CreateGeomap()
{
	gmmlObject = new gmml::Geo3DMap;
}

bool  Geo3DMapXMLReader::ReadNode(xmlNodePtr curNode)
{

	if (xmlStrcmp(curNode->name, BAD_CAST "Geo3DMap"))
	{
		return false;
	}

	xmlChar* tmp_str;        //临时字符串变量


	curNode = curNode->xmlChildrenNode;
	xmlNodePtr NodePtr_replica = curNode;
	gmml::Geo3DMap* map = NULL;
	CreateGeomap();
	map = (gmml::Geo3DMap*)gmmlObject;
	while (curNode != NULL)
	{
		//取出Geo3DMap节点中的内容
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Name")))
		{
			gmmlProgressing();
			tmp_str = xmlNodeGetContent(curNode);
			char* out = 0;
			if (std::string(mXMLCoding) == "UTF-8")
				out = ConvertEnc("UTF-8", "gb2312", (char*)tmp_str);
			std::string sr;
			if (out)
				sr = std::string((char*)out);
			else
				sr = std::string((char*)tmp_str);
			map->setName(sr);

			xmlFree(tmp_str);
		}
		else if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Layers")))
		{
			gmmlProgressing();
			xmlNodePtr layerNode = curNode->xmlChildrenNode;
			while (layerNode)
			{
				if ((!xmlStrcmp(layerNode->name, (const xmlChar *)"Layer")))
				{
					Geo3DLayerXMLReader GeoLayerReader;
					if (map)
						GeoLayerReader.ReadGeoLayer(layerNode);
					///@brief 将GeoLayer加入到新GeoLayer中
					if (map)
						map->AddGeoLayer((gmml::Geo3DLayer*)GeoLayerReader.GetGeoLayer());
				}
				layerNode = layerNode->next;
			}
		}
		else if (xmlStrcmp(curNode->name, (const xmlChar *)"text"))
		{
			gmmlProgressing();
			tmp_str = xmlNodeGetContent(curNode);
			char* out = NULL;
			if (std::string(mXMLCoding) == "UTF-8")
				out = ConvertEnc("UTF-8", "GB2312", (char*)tmp_str);
			std::string sr;
			if (out)
				sr = std::string((char*)out);
			else
				sr = std::string((char*)tmp_str);
			AttributeValue attr_desc;
			attr_desc.fieldName = "Description";
			attr_desc.fieldValue = sr;
			map->AddAttributeValue(attr_desc);
			xmlFree(tmp_str);
		}

		curNode = curNode->next;
	}

	return true;

}
