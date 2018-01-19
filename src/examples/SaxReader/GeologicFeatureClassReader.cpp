//#include "StdAfx.h"
#include "GeologicFeatureClassReader.h"

#include "FeatureClass.h"
#include "GeologicFeature.h"
#include "FieldReader.h"
#include "GeoFeatureReader.h"

using gmml::GeologicFeatureClass;
using gmml::GeologicFeature;


int ReadGeologicFeatureClass(void* pData,xmlTextReaderPtr reader,int binary)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
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
			if (!xmlStrcmp(name,BAD_CAST"GeoFeatureClass"))
			{
				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					((GeologicFeatureClass*)pData)->setID(out);
				}
				
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:name"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					((GeologicFeatureClass*)pData)->setName(out);
				}
				
			}
			else if (!xmlStrcmp(name,BAD_CAST"swe:field"))
			{
				AttributeValue value;
				ReadField(value,reader);
				((GeologicFeatureClass*)pData)->AddFeatureClassSchema(value);
			}
			else if (!xmlStrcmp(name,BAD_CAST"GeoFeature"))
			{
				GeologicFeature* pGeologicFeature = new GeologicFeature();
				ReadGeoFeature(pGeologicFeature,reader,binary);
				((GeologicFeatureClass*)pData)->AddGeologicFeature(pGeologicFeature);
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"FeatureClass"))
			{
				return 0;
			}

		}
		xmlFree(name);
		nRet = xmlTextReaderRead(reader);
	}
}
