#include "GeologicFeatureXMLReader.h"
#include "GeologicFeature.h"
#include "FeatureClass.h"
#include "MappedFeature.h"
#include <libxml/parser.h>//"parser.h"//  [1/5/2018 BigDog]
#include <fstream>
#include "iconv.h"
#include <string>
GeologicFeatureXMLReader::GeologicFeatureXMLReader(void)
{
	GeologicFeature_=0;
}
GeologicFeatureXMLReader::~GeologicFeatureXMLReader(void)
{
	if(GeologicFeature_->getContainer() == NULL)
	   delete GeologicFeature_;
}

///@brief 获取GeologicFeature
///@retun 返回GeologicFeature父指针
///@author yang hongjun
///@data 2013.11.12
gml::AbstractGML* GeologicFeatureXMLReader::GetGeologicFeature()
{
	return GeologicFeature_;
}
///@ brief 创建新GeologicFeature
///@ author yang hongjun
///@data 2013.11.12
void GeologicFeatureXMLReader::CreateGeologicFeature()
{
	GeologicFeature_=new gmml::GeologicFeature;
}

bool GeologicFeatureXMLReader::ReadGeologicFeature(xmlNode* geologicFeatureNode)
{
	xmlChar* tmp_str;
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动)      
	curNode=geologicFeatureNode;      //将由上个继承来的GeologicFeature的指针赋给curNode
	curNode = curNode->xmlChildrenNode;
	xmlNodePtr NodePtr_replica = curNode;
	gmml::GeologicFeature* geologicfeature = NULL;
	CreateGeologicFeature();
	geologicfeature=(gmml::GeologicFeature*)GeologicFeature_;
	while(curNode != NULL) 
	{
		
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Name"))) 
	    {
		tmp_str = xmlNodeGetContent(curNode);
		char* out = ConvertEnc("utf-8","gb2312", (char*)tmp_str);
		if(out)
			geologicfeature->setName(out);
		else
			geologicfeature->setName((char*)tmp_str);


		xmlFree(tmp_str); 
		} 

		if(!xmlStrcmp(curNode->name,(const xmlChar*)"MappedFeature"))
		{
			

		}

	}




	return true;
}