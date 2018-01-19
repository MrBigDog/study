#include "GeoModelXMLReader.h"
#include "GeoModel.h"
#include "MetaDataXMLReader.h"
#include "FeatureClassXMLReader.h"
#include "FeatureRelationXMLReader.h"

#include <fstream>
#include "iconv.h"
#include <string>
#include "gmmlSystem.h"
using namespace gmml;

GeoModelXMLReader::GeoModelXMLReader(void)
{
	 
}

GeoModelXMLReader::~GeoModelXMLReader(void)
{
	
}
///@brief 获取Geomodel对象
///@param 
///@author yang hongjun
///date 2013.11.10
gml::AbstractGML* GeoModelXMLReader::GetGeomodel()
{
	return gmmlObject;
}

///@brief 创建一个新的Geomodel对象
///@param 
///@author yang hongjun
///@date 2013.11.10
void GeoModelXMLReader::CreateGeomodel()
{
	gmmlObject = new gmml::GeoModel;
}

 





  ///@brief  读取GeoModel
  ///@param DocName引用上一节从model中解析的文件名的指针
  ///@return 如果读取成功，返回true；否则返回false
  ///@author yang hongjun
  ///@date   2013.11.14
  bool GeoModelXMLReader::ReadNode(xmlNodePtr curNode)
  {	 
	 
	  xmlChar* tmp_str;        //临时字符串变量
     
	  if(curNode == NULL)
	  { 		 
		  return false; 
	  } 

	  if(xmlStrcmp(curNode->name, BAD_CAST "GeoModel")) 
	  {		 
		  return false;
	  } 

	  curNode=curNode->xmlChildrenNode;
	  xmlNodePtr  NodePtr_replica = curNode;
	  CreateGeomodel();
	  gmml::GeoModel* model=(gmml::GeoModel*)gmmlObject;

	  while(curNode!=NULL)
	  {
		  ///读取name节点
		  if (!xmlStrcmp(curNode->name, (const xmlChar *)"Name"))
		  {
			  gmmlProgressing();
			  tmp_str = xmlNodeGetContent(curNode);
			  char* out = NULL;
			  if(std::string(mXMLCoding) == "UTF-8")
				  out = ConvertEnc("UTF-8","GB2312", (char*)tmp_str);
			  std::string sr;
			  if (out)
				  sr=std::string((char*)out);
			  else
				  sr=std::string((char*)tmp_str);
			  model->setName(sr);
			  xmlFree(tmp_str);  
		  }
		  else if (!xmlStrcmp(curNode->name, (const xmlChar *)"Type"))
		  {
			  gmmlProgressing();
			  tmp_str = xmlNodeGetContent(curNode);
			  char* out =0;
			  if(std::string(mXMLCoding) == "UTF-8")
				  out = ConvertEnc("UTF-8","GB2312", (char*)tmp_str);
			  std::string sr;
			  if (out)
				  sr=std::string((char*)out);
			  else
				  sr=std::string((char*)tmp_str);

			  model->SetType(sr);

			  xmlFree(tmp_str); 
		  }

		  else if((!xmlStrcmp(curNode->name, (const xmlChar *)"Metadata"))) 
		  {	
			  gmmlProgressing();
			  MetaDataXMLReader metadataReader;
			  if(model)
				  metadataReader.ReadGeoModelMetaData(model, curNode);
		  } 

		  else if(!xmlStrcmp(curNode->name, (const xmlChar *)"FeatureClasses"))
		  {
			  gmmlProgressing();
			  gmmlShowProgress("FeatureClasses");
			  xmlNodePtr curFeatureClasseNode = curNode->xmlChildrenNode;
			  while(curFeatureClasseNode)
			  {
				  if(!xmlStrcmp(curFeatureClasseNode->name, (const xmlChar *)"FeatureClass"))
				  {
					  FeatureClassXMLReader featureclassReader;
					  if(model)
						  featureclassReader.ReadFeatureClass(curFeatureClasseNode);

					  if(model)
					  {
						  if((gmml::GeologicFeatureClass*)featureclassReader.GetFeatureClass())
							model->AddFeatureClass((gmml::GeologicFeatureClass*)featureclassReader.GetFeatureClass());
					  }
						  
				  }
				  curFeatureClasseNode=curFeatureClasseNode->next;
			  }

		  }

		  else if(!xmlStrcmp(curNode->name,(const xmlChar*)"FeatureRelationship"))
		  {
			  gmmlProgressing();
			  gmmlShowProgress("FeatureRelationship");
			  xmlNodePtr curRelationNode=curNode->xmlChildrenNode;
			  while(curRelationNode)
			  {
				  if(!xmlStrcmp(curRelationNode->name,(const xmlChar*)"Relation"))
				  {
					  FeatureRelationXMLReader featurerelationReader;
					  featurerelationReader.ReadNode(curRelationNode);
					  if(model)
						  model->AddFeatureRelation((gmml::FeatureRelation*)featurerelationReader.GetReadObject()); 
				  }

				  curRelationNode=curRelationNode->next;
			  }	
		  }

		  curNode=curNode->next;
	  }
	  return true;
  }