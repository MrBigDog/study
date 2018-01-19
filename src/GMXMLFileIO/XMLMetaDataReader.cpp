#include "XMLMetaDataReader.h"
#include "Geo3DProject.h"
#include "Geo3DMap.h"
#include "Geo3DProjectMetaData.h"
#include "Geo3DMapMetaData.h"

///@brief 构造函数
XMLMetaDataReader::XMLMetaDataReader(void)
{

}

///@brief 析构函数
XMLMetaDataReader::~XMLMetaDataReader(void)
{

}

///@brief  读取GeoProject的元数据
///@params Geo3DProject* geoproject 该元数据对应的Geo3DProject
///        xmlNode* metadataNode 该元数据所在的XML结点
///@return 
///@author zhaowei
///@date   2013.11.1
void XMLMetaDataReader::ReadGeoProjectMetaData(gmml::Geo3DProject* geoproject, xmlNode* metadataNode)
{
	std::string ChildrenNodeNames[]={
		"Name",\
		"Description",\
		"Field"\
	};

	int ChildrenNodeNamesCount = 3;

	xmlNodePtr curNode;		//当前结点
	curNode = metadataNode;	//给当前结点赋值
	xmlNodePtr curNode_replica = curNode;	//为当前结点保存一份副本
	xmlChar* tmp_string;	//临时字符串变量
	gmml::Geo3DProjectMetaData* metadata = geoproject->GetGeoProjectMetaData();

	if(!xmlStrcmp(curNode->name, (const xmlChar*)"Metadata"))
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode && metadata)
		{
			//获取元数据中的各个字段。因为XML文件中各个标签的位置顺序可以颠倒，所以需要遍历一遍各个标签。
			for(int i = 0; i < ChildrenNodeNamesCount; i++)
			{
				if(!xmlStrcmp(curChildNode->name, (const xmlChar*)ChildrenNodeNames[i].c_str()))
				{				
					tmp_string = xmlNodeGetContent(curChildNode);
					//将字段内容中的空白字符清除掉
					int left_count = TrimLeft((char*)tmp_string);
					int right_count = TrimRight((char*)tmp_string);
					int len = strlen((char*)tmp_string);
					std::string value = (char*)tmp_string;
					value = value.substr(left_count, len-left_count-right_count);

					AttributeValue curNodeValue;
					curNodeValue.fieldName = ChildrenNodeNames[i];
					curNodeValue.fieldValue = value; 

					metadata->AddAttributeValue(curNodeValue);
					xmlFree(tmp_string);

					break;
				}
			}			

			//如果当前结点的下一结点为text标签，则跳过
			curNode = curNode->next;
			if ((xmlStrcmp(curNode->name, (const xmlChar *)"text"))) 
				curNode = curNode->next;
		}
	}
}


///@brief  读取GeoMap的元数据
///@params Geo3DMap* geomap 该元数据对应的Geo3DMap
///        xmlNode* metadataNode 该元数据所在的XML结点
///@return 
///@author zhaowei
///@date   2013.11.1
void XMLMetaDataReader::ReadGeoMapMetaData(gmml::Geo3DMap* geomap, xmlNode* metadataNode)
{
	std::string ChildrenNodeNames[]={
		"Name",\
		"Description",\
		"Owner",\
		"Date",\
		"ToolName",\
		"ToolVersion"
	};

	int ChildrenNodeNamesCount = 6;

	xmlNodePtr curNode;		//当前结点
	curNode = metadataNode;	//给当前结点赋值
	xmlNodePtr curNode_replica = curNode;	//为当前结点保存一份副本
	xmlChar* tmp_string;	//临时字符串变量
	gmml::Geo3DMapMetaData* metadata = geomap->GetGeoMapMetaData();

	if(!xmlStrcmp(curNode->name, (const xmlChar*)"Metadata"))
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode && metadata)
		{
			//获取元数据中的各个字段。因为XML文件中各个标签的位置顺序可以颠倒，所以需要遍历一遍各个标签。
			for(int i = 0; i < ChildrenNodeNamesCount; i++)
			{
				if(!xmlStrcmp(curChildNode->name, (const xmlChar*)ChildrenNodeNames[i].c_str()))
				{				
					tmp_string = xmlNodeGetContent(curChildNode);
					//将字段内容中的空白字符清除掉
					int left_count = TrimLeft((char*)tmp_string);
					int right_count = TrimRight((char*)tmp_string);
					int len = strlen((char*)tmp_string);
					std::string value = (char*)tmp_string;
					value = value.substr(left_count, len-left_count-right_count);

					AttributeValue curNodeValue;
					curNodeValue.fieldName = ChildrenNodeNames[i];
					curNodeValue.fieldValue = value; 

					metadata->AddAttributeValue(curNodeValue);
					xmlFree(tmp_string);

					break;
				}
			}			

			//如果当前结点的下一结点为text标签，则跳过
			curNode = curNode->next;
			if ((xmlStrcmp(curNode->name, (const xmlChar *)"text"))) 
				curNode = curNode->next;
		}
	}
}
