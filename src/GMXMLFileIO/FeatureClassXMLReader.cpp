#include "FeatureClassXMLReader.h"
#include "GeoFeatureXMLReader.h"
#include "FeatureClass.h"

#include <fstream>
#include "iconv.h"
#include <string>

FeatureClassXMLReader::FeatureClassXMLReader(void)
{
	FeatureClass_=NULL;
}


FeatureClassXMLReader::~FeatureClassXMLReader(void)
{
	if (FeatureClass_)
	{
		if(FeatureClass_->getContainer() == NULL)
			delete FeatureClass_;
	}
}

///@brief 获取FeatureClass对象
///@param 
///@author yang hongjun
///date 2013.11.10
gml::AbstractGML* FeatureClassXMLReader::GetFeatureClass()
{
	return FeatureClass_;
}
///@brief 创建一个新的FeatureClass对象
///@param
///@author yang hongjun
///@date 2013.11.09
void FeatureClassXMLReader::CreateFeatureClass()
{
	if(!FeatureClass_)
		FeatureClass_ = new gmml::GeologicFeatureClass;
}

///@brief  读取FeatureClass
///@param featureclassNode为FeatureClass的节点指针
///@return 如果读取成功，返回true；否则返回false
///@author  zhaowei
///@date   	2013.11.20
bool FeatureClassXMLReader::ReadFeatureClass(xmlNode* featureclassNode)
{
	xmlNodePtr curNode = featureclassNode->xmlChildrenNode;
	while (curNode && (xmlStrcmp(curNode->name, (const xmlChar *)"GeoFeatureClass") ))
		curNode = curNode->next;
	if (!curNode)
		return false;

	xmlChar * tmp_str;
	CreateFeatureClass();
	gmml::GeologicFeatureClass* featureclass = (gmml::GeologicFeatureClass*)FeatureClass_;

	///读取FeatureClass的id
	
	if(xmlHasProp(curNode, BAD_CAST "id"))
	{
		xmlAttrPtr attrPtr = curNode->properties;
		if(!xmlStrcmp(attrPtr->name, BAD_CAST "id"))
		{
			xmlChar* attr_str = xmlGetProp(curNode, BAD_CAST "id");
			char* out = NULL;
			if(std::string(mXMLCoding) == "UTF-8")
				out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
			std::string sr = "";
			if (out)
				sr=std::string((char*)out);
			else
				sr=std::string((char*)attr_str);

			featureclass->setID(sr);
			xmlFree(attr_str);                                             
		}
	}
	

	curNode = curNode->xmlChildrenNode;
	while(curNode != NULL)
	{		
		///读取name节点
		if(!xmlStrcmp(curNode->name, (const xmlChar *)"name"))
		{
			tmp_str = xmlNodeGetContent(curNode);
			char* out = 0;
			if(std::string(mXMLCoding) == "UTF-8")
				out =  ConvertEnc("UTF-8","GB2312", (char*)tmp_str);
			std::string sr;
			if (out)
				sr=std::string((char*)out);
			else
				sr=std::string((char*)tmp_str);
			
			featureclass->setName(sr);
			
			 xmlFree(tmp_str); 
		}
		else if(!xmlStrcmp(curNode->name, (const xmlChar *)"Schema"))
		{
			xmlNodePtr curSchemaNode = curNode->xmlChildrenNode;
			while(curSchemaNode)
			{
				if(!xmlStrcmp(curSchemaNode->name, (const xmlChar *)"field"))
				{	
					xmlAttrPtr attrPtr = curSchemaNode->properties;
					if(xmlHasProp(curSchemaNode, BAD_CAST "name"))
					{
						xmlChar* attr_str = xmlGetProp(curSchemaNode, BAD_CAST "name");
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
						//value.fieldType = (std::string)((char*)curSchemaNode->children->next->name);
						//if(xmlStrcmp(curFieldNode->children->next->name, (const xmlChar*)"text"))

						xmlNodePtr curFieldChildNode = curSchemaNode->children;
						while(curFieldChildNode)
						{
							if(xmlStrcmp(curFieldChildNode->name, (const xmlChar *)"swe:Text"))
							{
								value.fieldType =(char*) curFieldChildNode->name;

								/*xmlNodePtr valuenode = curFieldChildNode->children;

								while(valuenode)
								{
									if(!xmlStrcmp(valuenode->name, (const xmlChar *)"value"))
									{
										value.fieldValue = (char*)xmlNodeGetContent(valuenode);
									}

									valuenode = valuenode->next; 

								}*/

							}	

							if(xmlStrcmp(curFieldChildNode->name, (const xmlChar *)"swe:Double"))
							{
								value.fieldType =(char*) curFieldChildNode->name;

								/*xmlNodePtr valuenode = curFieldChildNode->children;

								while(valuenode)
								{
									if(!xmlStrcmp(valuenode->name, (const xmlChar *)"value"))
									{
										value.fieldValue = (char*)xmlNodeGetContent(valuenode);
									}

									valuenode = valuenode->next; 

								}*/

							}


							if(xmlStrcmp(curFieldChildNode->name, (const xmlChar *)"swe:Count"))
							{
								value.fieldType =(char*) curFieldChildNode->name;

								/*xmlNodePtr valuenode = curFieldChildNode->children;

								while(valuenode)
								{
									if(!xmlStrcmp(valuenode->name, (const xmlChar *)"value"))
									{
										value.fieldValue = (char*)xmlNodeGetContent(valuenode);
									}

									valuenode = valuenode->next; 

								}*/

							}
							curFieldChildNode = curFieldChildNode->next; 
						}
						featureclass->AddFeatureClassSchema(value);
						xmlFree(attr_str);       
					}
				}
				curSchemaNode = curSchemaNode->next;
			}
		}
		else if(!xmlStrcmp(curNode->name, (const xmlChar *)"Features"))
		 {
			 xmlNodePtr curFeaturesNode=curNode->xmlChildrenNode;
			 while(curFeaturesNode)
			 {
				 if(!xmlStrcmp(curFeaturesNode->name, (const xmlChar *)"Feature"))
				 {
					 xmlNodePtr geoFeaturesNode = curFeaturesNode->xmlChildrenNode;
					 while(geoFeaturesNode)
					 {
						 if(!xmlStrcmp(geoFeaturesNode->name, (const xmlChar *)"GeoFeature"))
						 {
							GeoFeatureXMLReader geofeatureReader;
							geofeatureReader.ReadNode(geoFeaturesNode);
							if(featureclass)
								featureclass->AddGeologicFeature((gmml::GeologicFeature*)geofeatureReader.GetGeoFeature());
						 }
						 geoFeaturesNode = geoFeaturesNode->next;
					 } 
				 }
				  curFeaturesNode = curFeaturesNode->next;
			 }			
		 }
		  curNode = curNode->next;
	}
	return true;
}