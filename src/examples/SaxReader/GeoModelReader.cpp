//#include "StdAfx.h"
#include "GeoModelReader.h"

#include "GeoModel.h"
#include "FeatureClass.h"
#include "GeoModelMetaData.h"
#include "FeatureRelation.h"

#include "GeologicFeatureClassReader.h"
#include "FeatureRelationReader.h"
#include "gmmlSystem.h"

//using gmml::GeoModel;
//using gmml::GeologicFeatureClass;
//using gmml::GeoModelMetaData;
//using gmml::FeatureRelation;


int ReadGeoModel(void* pData, const char* pGeoModelFile, int binary)
{
	InitCharSet(pGeoModelFile);

	xmlTextReaderPtr reader;
	reader = xmlReaderForFile(pGeoModelFile, gCharSet, XML_PARSE_NOBLANKS);
	if (reader != NULL)
	{
		int nRet = 1;
		xmlChar* name = NULL, *value = NULL;

		nRet = xmlTextReaderRead(reader);
		name = xmlTextReaderName(reader);

		if (!xmlStrcmp(name, BAD_CAST"GeoModel"))
		{
			while (nRet == 1)
			{
				name = xmlTextReaderName(reader);
				if (!xmlStrcmp(name, BAD_CAST"#text"))
				{
					nRet = xmlTextReaderRead(reader);
					continue;
				}

				if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
				{
					gmmlProgressing();
					if (!xmlStrcmp(name, BAD_CAST"Name"))
					{
						char*out = ReadValueAndConvert(reader);
						if (out)
						{
							((gmml::GeoModel*)pData)->setName(out);
						}


					}
					else if (!xmlStrcmp(name, BAD_CAST"Type"))
					{
						gmmlProgressing();
						char*out = ReadValueAndConvert(reader);
						if (out)
						{
							((gmml::GeoModel*)pData)->SetType((const char*)out);
						}
					}
					else if (!xmlStrcmp(name, BAD_CAST"Metadata"))
					{
						gmmlProgressing();
						gmml::GeoModelMetaData* pMetaData = ((gmml::GeoModel*)pData)->GetGeoModelMetaData();
						ReadMetaData(pMetaData, reader);
					}
					else if (!xmlStrcmp(name, BAD_CAST"FeatureClass"))
					{
						gmmlProgressing();
						gmmlShowProgress("FeatureClasses");
						gmml::GeologicFeatureClass* pGeologicFeatureClass = new gmml::GeologicFeatureClass();

						ReadGeologicFeatureClass(pGeologicFeatureClass, reader, binary);

						((gmml::GeoModel*)pData)->AddFeatureClass(pGeologicFeatureClass);
					}
					else if (!xmlStrcmp(name, BAD_CAST"FeatureRelationship"))
					{
						gmmlProgressing();
						gmmlShowProgress("FeatureRelationship");
						if (!xmlStrcmp(name, BAD_CAST"FeatureRelationship"))
						{
							//FeatureRelation* pFeatureRelation = new FeatureRelation();
							ReadFeatureRelation(pData, reader);

							//((GeoModel*)pData)->AddFeatureRelation(pFeatureRelation); 
							return 0;
						}
					}

				}
				else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
				{
					if (!xmlStrcmp(name, BAD_CAST"GeoModel"))
					{
						xmlFreeTextReader(reader);

						return 0;
					}

				}

				nRet = xmlTextReaderRead(reader);
			}
		}
	}

}

int ReadMetaData(void* pData, xmlTextReaderPtr reader)
{
	int nRet = 1;
	nRet = xmlTextReaderRead(reader);
	xmlChar* name = NULL;

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{

			if (xmlStrcmp(name, BAD_CAST"text"))
			{
				int left_count, right_count, len;
				char *tmp_str = (char*)xmlTextReaderReadString(reader);
				char *out = ReadValueAndConvert(reader);

				std::string value;
				if (out)
				{
					left_count = TrimLeft(out);
					right_count = TrimRight(out);
					len = strlen(out);
					value = (std::string)out;
				}
				else
				{
					left_count = TrimLeft(tmp_str);
					right_count = TrimRight(tmp_str);
					len = strlen(tmp_str);
					value = (std::string)(tmp_str);
				}

				value = value.substr(left_count, len - left_count - right_count);

				AttributeValue curNodeValue;
				curNodeValue.fieldName = (std::string)((char*)name);
				curNodeValue.fieldValue = value;
				((gmml::GeoModelMetaData*)pData)->AddAttributeValue(curNodeValue);
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"Metadata"))
			{
				return 0;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}



	return 0;
}