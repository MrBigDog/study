#include "Geo3DProjectXMLReader.h"
#include "Geo3DProject.h"
#include "Geo3DProjectMetaData.h"
#include "MetaDataXMLReader.h"
#include "GeometryXMLReader.h"

#include <fstream>
#include "iconv.h"
#include <string>
#include "Geo3DMapXMLReader.h"	
#include "GeoModelXMLReader.h"	
#include "gmmlSystem.h"
namespace gmml
{
	class Geo3DProject;
	class Geo3DProjectMetaData;
	class Geo3DMap;
	class Geo3DMapMetaData;
}

///@brief  构造函数
///@author zhaowei
///@date   2013.11.04
Geo3DProjectXMLReader::Geo3DProjectXMLReader()
{
	Project_ = NULL;
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.04
Geo3DProjectXMLReader::~Geo3DProjectXMLReader()
{

}

///@brief  获取Geo3DProject
///@param 
///@return 返回指向Geo3DProject父类的指针
///@author zhaowei
///@date   2013.11.04
gml::AbstractGML* Geo3DProjectXMLReader::GetGeoProject()
{
	return Project_;
}

///@brief  创建新的Geo3DProject对象
///@param  
///@return 
///@author zhaowei
///@date   2013.11.04
void Geo3DProjectXMLReader::CreateProject()
{
	Project_ = new gmml::Geo3DProject;
}

///@brief  读取Geo3DProject
///@param  projectFile 包含待读取Geo3DProject的文件
///@return 如果读取成功，返回true；否则返回false
///@author zhaowei
///@date   2013.11.04
bool Geo3DProjectXMLReader::ReadFile(const char *DocName, const std::string charSet, int binary)
{
	GeometryXMLReader::BinaryExport = binary;

	InitCharSet(DocName);
	xmlDocPtr doc;           //定义解析文档指针
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	xmlChar* tmp_str;        //临时字符串变量

	gmmlDisplayErrorandWarningText(DocName);


	std::string codeType;// = mXMLCoding;
	if (charSet == "")
	{
		codeType = gCharSet;
	}
	else
	{
		codeType = charSet;
	}

	doc = xmlReadFile((char*)DocName, codeType.c_str(), XML_PARSE_RECOVER); //解析文件
//	strcpy(mXMLCoding,codeType.c_str());
	//检查解析文档是否成功，如果不成功，libxml将指向一个注册的错误并停止。

	//一个常见错误是不适当的编码。XML标准文档除了用UTF-8或UTF-16外还可用其它编码保存。

	//如果文档是这样，libxml将自动地为你转换到UTF-8。更多关于XML编码信息包含在XML标准中.

	if (doc == NULL)
	{
		return false;
	}

	curNode = xmlDocGetRootElement(doc); //确定文档根元素
	if (curNode == NULL)
	{
		xmlFreeDoc(doc);
		return false;
	}

	if (xmlStrcmp(curNode->name, BAD_CAST "Geo3DProject"))
	{
		xmlFreeDoc(doc);
		return false;
	}

	curNode = curNode->xmlChildrenNode;
	xmlNodePtr NodePtr_replica = curNode;
	gmml::Geo3DProject* project = NULL;
	gmml::Geo3DProjectMetaData* project_metadata = new gmml::Geo3DProjectMetaData;
	std::vector<std::string> aryFiles;

	if (!Project_)
	{
		CreateProject();
		project = (gmml::Geo3DProject*)Project_;
	}

	gmmlStartProgress(100);
	std::string srPath(DocName);
	int apath = srPath.rfind("\\");
	project->setIOPath(srPath.substr(0, apath));
	srPath = srPath.substr(0, apath) + "\\";
	while (curNode != NULL)
	{
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Name")))
		{
			tmp_str = xmlNodeGetContent(curNode);

			char* out = 0;
			if (std::string(mXMLCoding) == "UTF-8")
				out = ConvertEnc("UTF-8", "GB2312", (char*)tmp_str);
			std::string sr;
			if (out)
				sr = std::string((char*)out);
			else
				sr = std::string((char*)tmp_str);
			project->setName(sr);

			xmlFree(tmp_str);
		}
		else
			/// 解析project文件中的Description字段，作为projectMetaData的一个属性
			if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Description")))
			{
				tmp_str = xmlNodeGetContent(curNode);
				char* out = 0;
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
				project_metadata->AddAttributeValue(attr_desc);
				project->setDescription(sr);
				xmlFree(tmp_str);
			}

			else if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Maps")))
			{
				xmlNodePtr MapNode = curNode->xmlChildrenNode;
				while (MapNode)
				{
					if (!xmlStrcmp(MapNode->name, (const xmlChar*)"Map"))
					{
						xmlNodePtr geo3DMapNode = MapNode->xmlChildrenNode;
						while (geo3DMapNode)
						{
							if (!xmlStrcmp(geo3DMapNode->name, (const xmlChar*)"include"))
							{
								NodePtr_replica = geo3DMapNode;
								xmlAttrPtr attrPtr = NodePtr_replica->properties;
								while (attrPtr)
								{
									if (!xmlStrcmp(attrPtr->name, BAD_CAST "href"))
									{
										xmlChar* attr_str = xmlGetProp(NodePtr_replica, BAD_CAST "href");
										char* out = 0;
										if (std::string(mXMLCoding) == "UTF-8")
											out = ConvertEnc("UTF-8", "GB2312", (char*)attr_str);

										std::string filepath;

										if (out)
										{
											filepath = out;
										}
										else
										{
											filepath = (char*)attr_str;
										}

										if (project)
										{
											gmmlShowProgress(filepath.c_str());
											filepath = srPath + filepath;
											Geo3DMapXMLReader geo3DmapReader;
											geo3DmapReader.ReadFile(filepath.c_str(), charSet);

											project->AddGeoMap((gmml::Geo3DMap*)geo3DmapReader.GetReadObject());
										}

										xmlFree(attr_str);
									}
									attrPtr = attrPtr->next;
								}

							}
							else
								if (!xmlStrcmp(geo3DMapNode->name, (const xmlChar*)"Geo3DMap"))
								{
									Geo3DMapXMLReader geo3DmapReader;
									if (project)
										geo3DmapReader.ReadNode(geo3DMapNode);
									if (project)
										project->AddGeoMap((gmml::Geo3DMap*)geo3DmapReader.GetGeomap());
								}

							geo3DMapNode = geo3DMapNode->next;
						}
					}
					MapNode = MapNode->next;
				}
			}
			else if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Models")))
			{
				xmlNodePtr ModelNode = curNode->xmlChildrenNode;
				while (ModelNode)
				{
					if (!xmlStrcmp(ModelNode->name, (const xmlChar*)"Model"))
					{
						xmlNodePtr geo3DModelNode = ModelNode->xmlChildrenNode;
						while (geo3DModelNode)
						{
							if (!xmlStrcmp(geo3DModelNode->name, (const xmlChar*)"include"))
							{
								NodePtr_replica = geo3DModelNode;
								xmlAttrPtr attrPtr = NodePtr_replica->properties;
								while (attrPtr)
								{
									if (!xmlStrcmp(attrPtr->name, BAD_CAST "href"))
									{
										xmlChar* attr_str = xmlGetProp(NodePtr_replica, BAD_CAST "href");
										char* out = 0;
										if (std::string(mXMLCoding) == "UTF-8")
											out = ConvertEnc("UTF-8", "GB2312", (char*)attr_str);

										std::string filepath;
										if (out)
										{
											filepath = out;
										}
										else
										{
											filepath = (char*)attr_str;
										}

										if (project)
										{
											//	gmmlStartProgress(6);
											gmmlShowProgress(filepath.c_str());
											filepath = srPath + filepath;
											GeoModelXMLReader geoModelReader;
											geoModelReader.ReadFile(filepath.c_str(), charSet);
											project->AddGeoModel((gmml::GeoModel*)geoModelReader.GetReadObject());
										}

										xmlFree(attr_str);
									}
									attrPtr = attrPtr->next;
								}

							}
							else if (!xmlStrcmp(geo3DModelNode->name, (const xmlChar*)"GeoModel"))
							{
								GeoModelXMLReader geoModelReader;
								if (project)
									geoModelReader.ReadNode(geo3DModelNode);
								if (project)
									project->AddGeoModel((gmml::GeoModel*)geoModelReader.GetReadObject());
							}

							geo3DModelNode = geo3DModelNode->next;
						}
					}

					ModelNode = ModelNode->next;
				}
			}
			else if (xmlStrcmp(curNode->name, (const xmlChar*)"text"))
			{
				tmp_str = xmlNodeGetContent(curNode);
				char* out = 0;
				if (std::string(mXMLCoding) == "UTF-8")
					out = ConvertEnc("UTF-8", "GB2312", (char*)tmp_str);
				std::string sr;
				if (out)
					sr = std::string((char*)out);
				else
					sr = std::string((char*)tmp_str);
				AttributeValue attr_desc;
				attr_desc.fieldName = (std::string)((char*)curNode->name);
				attr_desc.fieldValue = sr;
				project_metadata->AddAttributeValue(attr_desc);
				xmlFree(tmp_str);
			}

		curNode = curNode->next;
	}
	project->SetGeoProjectMetaData(project_metadata);

	xmlFreeDoc(doc);

	gmmlStopProgress();
	gmmlDisplayErrorandWarningText("读取完毕");
	return true;
}