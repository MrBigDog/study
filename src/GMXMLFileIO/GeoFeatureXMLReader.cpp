#include "GeoFeatureXMLReader.h"
//#include "MappedFeatureXMLReader.h"
#include "GeometryXMLReader.h"
#include "GeologicFeature.h"
#include "FeatureClass.h"
#include "MappedFeature.h"

#include <fstream>
#include "iconv.h"
#include <string>
GeoFeatureXMLReader::GeoFeatureXMLReader(void)
{
	gmmlObject=0;
}
GeoFeatureXMLReader::~GeoFeatureXMLReader(void)
{
	if(gmmlObject->getContainer() == NULL)
	   delete gmmlObject;

	gmmlObject = 0;
}

///@brief 获取GeologicFeature
///@retun 返回GeologicFeature父指针
///@author yang hongjun
///@data 2013.11.12
gml::AbstractGML* GeoFeatureXMLReader::GetGeoFeature()
{
	return gmmlObject;
}
///@ brief 创建新GeologicFeature
///@ author yang hongjun
///@data 2013.11.12
void GeoFeatureXMLReader::CreateGeoFeature()
{
	if(!gmmlObject)
	gmmlObject = new gmml::GeologicFeature;
}

bool GeoFeatureXMLReader::ReadNode(xmlNode* geoFeatureNode)
{
	xmlChar* tmp_str;
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动)      
	curNode = geoFeatureNode;      //将由上个继承来的GeologicFeature的指针赋给curNode
	xmlNodePtr NodePtr_replica = curNode;
	gmml::GeologicFeature* GeologicFeature = NULL;
	CreateGeoFeature();
	GeologicFeature = (gmml::GeologicFeature*)gmmlObject;

	xmlAttrPtr attrPtr = curNode->properties;
	while(attrPtr)
	{
		if(!xmlStrcmp(attrPtr->name, BAD_CAST "id"))
		{
			xmlChar* attr_str = xmlGetProp(NodePtr_replica, BAD_CAST "id");
			char* out = 0;
			if(std::string(mXMLCoding) == "UTF-8") 
				out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
			std::string sr;
			if (out)
				sr=std::string(out);
			else
				sr=std::string((char*)attr_str);
			GeologicFeature->setID(sr);
			xmlFree(attr_str);                                             
		}			 
		attrPtr=attrPtr->next;			
	}

	curNode = curNode->xmlChildrenNode;

	while(curNode != NULL) 
	{
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"name"))) 
		{
			tmp_str = xmlNodeGetContent(curNode);
			char* out = 0;
			if(std::string(mXMLCoding) == "UTF-8")
				out = ConvertEnc("UTF-8","GB2312", (char*)tmp_str);
			std::string sr;
			if (out)
				sr=std::string((char*)out);
			else
				sr=std::string((char*)tmp_str);
			GeologicFeature->setName(sr);

			xmlFree(tmp_str); 
		} 
		///此部分还有Fields和Geometry没有解析
		else if(!xmlStrcmp(curNode->name, (const xmlChar *)"Fields"))
		{
			xmlNodePtr curFieldNode = curNode->children;
			while(curFieldNode)
			{
				if(!xmlStrcmp(curFieldNode->name, (const xmlChar *)"Field"))
				{
					xmlAttrPtr attrPtr = curFieldNode->properties;
					if(xmlHasProp(curFieldNode, BAD_CAST "Name"))
					{
						xmlChar* attr_str = xmlGetProp(curFieldNode, BAD_CAST "Name");
						char* out = NULL;
						if(std::string(mXMLCoding) == "UTF-8") 
							out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
						std::string sr;
						if (out)
							sr=std::string(out);
						else
							sr=std::string((char*)attr_str);
						AttributeValue value;
						value.fieldName = sr;
						//value.fieldType = (std::string)((char*)curFieldNode->children->next->name);
						//if(xmlStrcmp(curFieldNode->children->next->name, (const xmlChar*)"text"))

						xmlNodePtr curFieldChildNode = curFieldNode->children;
						while(curFieldChildNode)
						{
							if(!xmlStrcmp(curFieldChildNode->name, (const xmlChar *)"Text"))
							{
								value.fieldType =(char*) curFieldChildNode->name;

								xmlNodePtr valuenode = curFieldChildNode->children;

								while(valuenode)
								{
									if(!xmlStrcmp(valuenode->name, (const xmlChar *)"value"))
									{	
										char * szKey =  (char*)xmlNodeGetContent(valuenode);
										int leftc = TrimLeft((char*)szKey);
										int rightc = TrimRight((char*)szKey);
										int len = strlen((char*)szKey);

										char* out = NULL;
										if(std::string(mXMLCoding) == "UTF-8") 
											out =  ConvertEnc("UTF-8","GB2312", szKey);
										std::string sr;
										if (out)
											sr=std::string(out);
										else
											sr=std::string((char*)szKey);

										std::string FilterName = sr;
										FilterName = FilterName.substr(leftc,len -rightc-leftc);

										value.fieldValue  = FilterName;

									}
									
									valuenode = valuenode->next; 

								}

							}	

							if(!xmlStrcmp(curFieldChildNode->name, (const xmlChar *)"Count"))
							{
								value.fieldType =(char*) curFieldChildNode->name;

								xmlNodePtr valuenode = curFieldChildNode->children;

								while(valuenode)
								{
									if(!xmlStrcmp(valuenode->name, (const xmlChar *)"value"))
									{	
										char * szKey =  (char*)xmlNodeGetContent(valuenode);
										int leftc = TrimLeft((char*)szKey);
										int rightc = TrimRight((char*)szKey);
										int len = strlen((char*)szKey);

										char* out = NULL;
										if(std::string(mXMLCoding) == "UTF-8") 
											out =  ConvertEnc("UTF-8","GB2312", szKey);
										std::string sr;
										if (out)
											sr=std::string(out);
										else
											sr=std::string((char*)szKey);

										std::string FilterName = sr;

										value.fieldValue  = FilterName;

									}

									valuenode = valuenode->next; 

								}

							}		

							if(!xmlStrcmp(curFieldChildNode->name, (const xmlChar *)"Double"))
							{
								value.fieldType =(char*) curFieldChildNode->name;

								xmlNodePtr valuenode = curFieldChildNode->children;

								while(valuenode)
								{
									if(!xmlStrcmp(valuenode->name, (const xmlChar *)"value"))
									{	
										char * szKey =  (char*)xmlNodeGetContent(valuenode);
										int leftc = TrimLeft((char*)szKey);
										int rightc = TrimRight((char*)szKey);
										int len = strlen((char*)szKey);

										char* out = NULL;
										if(std::string(mXMLCoding) == "UTF-8") 
											out =  ConvertEnc("UTF-8","GB2312", szKey);
										std::string sr;
										if (out)
											sr=std::string(out);
										else
											sr=std::string((char*)szKey);

										std::string FilterName = sr;

										value.fieldValue  = FilterName;

									}

									valuenode = valuenode->next; 

								}

							}		
							curFieldChildNode = curFieldChildNode->next; 
						}

					 
						GeologicFeature->AddAttributeValue(value);
						xmlFree(attr_str);             
					}
				}
				curFieldNode = curFieldNode->next;
			}
		}
		else if(!xmlStrcmp(curNode->name, (const xmlChar *)"Geometry"))
		{
			GeometryXMLReader geometryreader;
			gmml::MappedFeature* mapped_feature = 0;
			if(GeologicFeature)
			     mapped_feature =  new gmml::MappedFeature;
			geometryreader.ReadNode(curNode, mapped_feature);

			if(GeologicFeature && mapped_feature)
				GeologicFeature->AddMappedFeature(mapped_feature);

		}
		curNode = curNode->next;
	}
	return true;
}