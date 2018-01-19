//#include "StdAfx.h"
#include "Geo3DMapReader.h"

#include "Geo3DMap.h"
#include "Geo3DLayer.h"
#include "Geo3DLayerReader.h"
#include "gmmlSystem.h"

int ReadGeo3DMap(void* pData,const char* pGeo3DMapFile)
{
	InitCharSet(pGeo3DMapFile);

	xmlTextReaderPtr reader;  
	reader=xmlReaderForFile(pGeo3DMapFile,gCharSet,XML_PARSE_NOBLANKS);
	if (reader != NULL)
	{
		int nRet = 1;
		xmlChar* name = NULL,*value = NULL;

		nRet = xmlTextReaderRead(reader);
		name = xmlTextReaderName(reader);
		if (xmlStrcmp(name,BAD_CAST"Geo3DMap"))
		{
			return 0;
		}

		nRet = xmlTextReaderRead(reader);
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
				if (!xmlStrcmp(name,BAD_CAST"Name"))
				{
					gmmlProgressing();
					char* out = ReadValueAndConvert(reader);
					if (out)
					{
						((gmml::Geo3DMap*)pData)->setName(out);
					}


				}
				else if (!xmlStrcmp(name,BAD_CAST"Description"))
				{
					char*out = ReadValueAndConvert(reader);
					if (out)
					{
						AttributeValue attr_desc;
						attr_desc.fieldName = "Description";
						attr_desc.fieldValue = out;

						((gmml::Geo3DMap*)pData)->AddAttributeValue(attr_desc);
					}


				}
				else if (!xmlStrcmp(name,BAD_CAST"Layer"))
				{
					gmmlProgressing();
					gmml::Geo3DLayer* pLayer = new gmml::Geo3DLayer();						
					ReadGeo3DLayer(pLayer,reader);
					((gmml::Geo3DMap*)pData)->AddGeoLayer(pLayer);
				}
				else if(xmlStrcmp(name, BAD_CAST"text"))
				{
					gmmlProgressing();
					char* out = ReadValueAndConvert(reader);
					if (out)
					{
						AttributeValue attr_desc;
						attr_desc.fieldName = "Description";
						attr_desc.fieldValue = out;
						((gmml::Geo3DMap*)pData)->AddAttributeValue(attr_desc);
					}
				}

			}
			else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
			{
				if (!xmlStrcmp(name,BAD_CAST"Geo3DMap"))
				{
					xmlFreeTextReader(reader); 

					return 0;
				}

			}

			nRet = xmlTextReaderRead(reader);
		}
	}
	

	return -1;
}
