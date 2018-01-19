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

///@brief  ���캯��
///@author zhaowei
///@date   2013.11.04
Geo3DProjectXMLReader::Geo3DProjectXMLReader()
{
	Project_ = NULL;
}

///@brief  ��������
///@author zhaowei
///@date   2013.11.04
Geo3DProjectXMLReader::~Geo3DProjectXMLReader()
{

}

///@brief  ��ȡGeo3DProject
///@param 
///@return ����ָ��Geo3DProject�����ָ��
///@author zhaowei
///@date   2013.11.04
gml::AbstractGML* Geo3DProjectXMLReader::GetGeoProject()
{
	return Project_;
}

///@brief  �����µ�Geo3DProject����
///@param  
///@return 
///@author zhaowei
///@date   2013.11.04
void Geo3DProjectXMLReader::CreateProject()
{
	Project_ = new gmml::Geo3DProject;
}

///@brief  ��ȡGeo3DProject
///@param  projectFile ��������ȡGeo3DProject���ļ�
///@return �����ȡ�ɹ�������true�����򷵻�false
///@author zhaowei
///@date   2013.11.04
bool Geo3DProjectXMLReader::ReadFile(const char *DocName, const std::string charSet, int binary)
{
	GeometryXMLReader::BinaryExport = binary;

	InitCharSet(DocName);
	xmlDocPtr doc;           //��������ĵ�ָ��
	xmlNodePtr curNode;      //������ָ��(����Ҫ��Ϊ���ڸ��������ƶ�) 
	xmlChar* tmp_str;        //��ʱ�ַ�������

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

	doc = xmlReadFile((char*)DocName, codeType.c_str(), XML_PARSE_RECOVER); //�����ļ�
//	strcpy(mXMLCoding,codeType.c_str());
	//�������ĵ��Ƿ�ɹ���������ɹ���libxml��ָ��һ��ע��Ĵ���ֹͣ��

	//һ�����������ǲ��ʵ��ı��롣XML��׼�ĵ�������UTF-8��UTF-16�⻹�����������뱣�档

	//����ĵ���������libxml���Զ���Ϊ��ת����UTF-8���������XML������Ϣ������XML��׼��.

	if (doc == NULL)
	{
		return false;
	}

	curNode = xmlDocGetRootElement(doc); //ȷ���ĵ���Ԫ��
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
			/// ����project�ļ��е�Description�ֶΣ���ΪprojectMetaData��һ������
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
	gmmlDisplayErrorandWarningText("��ȡ���");
	return true;
}