//#include "StdAfx.h"
#include "FeatureRelationReader.h"
#include "GeologicRelation.h"
#include "FeatureRelation.h"
#include "GeoModel.h"


using gmml::DefiningStructure;
using gmml::GeologicHistory;
using gmml::BoundaryRelation;
using gmml::AggregationRelation;
using gmml::FeatureRelation;
using gmml::GeoModel;


int ReadFeatureRelation(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,*pre_name = NULL;
	int nRet = 1;

	FeatureRelation*  pFeatureRelation = NULL;
	
	
	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if(!xmlStrcmp(name, BAD_CAST "BoundaryRelationship"))
			{
				pFeatureRelation = new gmml::BoundaryRelation;
				pFeatureRelation->setRelationship((char*)name);

				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					pFeatureRelation->setID(out);
				}
			}

			if(!xmlStrcmp(name, BAD_CAST "AggregationRelation"))
			{
				pFeatureRelation = new gmml::AggregationRelation;
				pFeatureRelation->setRelationship((char*)name);

				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					pFeatureRelation->setID(out);
				}
			}

			if(!xmlStrcmp(name, BAD_CAST "GeologicHistory"))
			{
				pFeatureRelation = new gmml::GeologicHistory;
				pFeatureRelation->setRelationship((char*)name);

				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					pFeatureRelation->setID(out);
				}
			}

			if(!xmlStrcmp(name, BAD_CAST "DefiningStructure"))
			{
				pFeatureRelation = new gmml::DefiningStructure;
				pFeatureRelation->setRelationship((char*)name);

				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					pFeatureRelation->setID(out);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:name"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					pFeatureRelation->setName(out);	
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"Source"))
			{
				ReadSource(pFeatureRelation,reader);
			}
			else if (!xmlStrcmp(name,BAD_CAST"Targets"))
			{
				ReadTargets(pFeatureRelation,reader);
			}

		}
		else if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"FeatureRelationship"))
			{
				return 0;
			}

			else if(!xmlStrcmp(name, BAD_CAST "BoundaryRelationship")||
				!xmlStrcmp(name, BAD_CAST "AggregationRelation")||
				!xmlStrcmp(name, BAD_CAST "DefiningStructure")||
				!xmlStrcmp(name, BAD_CAST "GeologicHistory"))
			{
				((GeoModel*)pData)->AddFeatureRelation(pFeatureRelation); 
			}
		}

		if (pre_name)
		{
			xmlFree(pre_name);
			pre_name = NULL;
		}

		if (name)
		{
			if (!pre_name)
			{
				pre_name = name;
			}

			
		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadTargets(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,*value = NULL;
	int nRet = 1;

	FeatureRelation* pFeatureRelation = (FeatureRelation*)pData;

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Feature"))
			{
				char* out = ReadAttrAndConvert(reader,"xlink:type");
				if (out)
				{
				}

				out = ReadAttrAndConvert(reader,"xlink:href");
				if (out)
				{
					std::string strOut(out); 
					if (strOut.length())
					{
						if(strOut.find("#")>=0)                   ///@因为Targets的id的开头是带有#的
							strOut = strOut.substr(1,strOut.size()-1);
						pFeatureRelation->addTargetRole(strOut);
					}
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Targets"))
			{
				return 0;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadSource(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,*value = NULL;
	int nRet = 1;

	FeatureRelation* pFeatureRelation = (FeatureRelation*)pData;

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Feature"))
			{
				char* out = ReadAttrAndConvert(reader,"xlink:type");
				if (out)
				{
				}

				out = ReadAttrAndConvert(reader,"xlink:href");
				if (out)
				{
					pFeatureRelation->setSourceRole(out);
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Source"))
			{
				return 0;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}
}