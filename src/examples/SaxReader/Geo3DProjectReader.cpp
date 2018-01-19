#include "Geo3DProjectReader.h"

#include "gmmlSystem.h"
#include "AbstractGML.h"
#include "Geo3DProject.h"
#include "Geo3DProjectMetaData.h"
#include "GeoModel.h"
#include "Geo3DMap.h"
#include "GeoModelReader.h"
#include "Geo3DMapReader.h"
#include "ShapeReader.h"

#include <string>

int ReadGeo3DProject(void* pData, const char* pGeo3DProjectFile, int binary)
{
	std::string strPathName;
	if (pGeo3DProjectFile)
	{
		std::string strPathFile(pGeo3DProjectFile);
		int nPos = strPathFile.find_last_of("\\");
		if (nPos != std::string::npos)
		{
			strPathName = strPathFile.substr(0, nPos);
			strPathName += "\\";
		}
	}

	InitCharSet(pGeo3DProjectFile);

	gmmlDisplayErrorandWarningText(pGeo3DProjectFile);

	xmlTextReaderPtr reader;
	reader = xmlReaderForFile(pGeo3DProjectFile, gCharSet, XML_PARSE_NOBLANKS);
	if (reader != NULL)
	{
		int nRet = -1;
		xmlChar* name = NULL, *value = NULL, *prev_name = NULL;

		nRet = xmlTextReaderRead(reader);
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
				gmmlStartProgress(100);
				if (!xmlStrcmp(name, BAD_CAST"Name"))
				{
					char* pOut = ReadValueAndConvert(reader);
					if (pOut)
					{
						((gmml::Geo3DProject*)pData)->setName(pOut);
					}
				}
				else if (!xmlStrcmp(name, BAD_CAST"Description"))
				{
					char* pOut = ReadValueAndConvert(reader);
					if (pOut)
					{
						AttributeValue attr_desc;
						attr_desc.fieldName = "Description";
						attr_desc.fieldValue = pOut;

						gmml::Geo3DProjectMetaData* pGeo3DProjectMetaData = new gmml::Geo3DProjectMetaData;
						pGeo3DProjectMetaData->AddAttributeValue(attr_desc);

						((gmml::Geo3DProject*)pData)->SetGeoProjectMetaData(pGeo3DProjectMetaData);
						((gmml::Geo3DProject*)pData)->setDescription(pOut);
					}
				}
				else if (!xmlStrcmp(name, BAD_CAST"xi:include"))
				{
					char* pOut = ReadAttrAndConvert(reader, "href");
					if (pOut)
					{
						std::string strTmp = strPathName;
						strTmp += pOut;

						if (!xmlStrcmp(prev_name, BAD_CAST"Map")
							|| !xmlStrcmp(prev_name, BAD_CAST"Geo3DMap"))
						{
							gmmlShowProgress(strTmp.c_str());
							gmml::Geo3DMap* pGeo3DMap = new gmml::Geo3DMap();
							ReadGeo3DMap(pGeo3DMap, strTmp.c_str());
							((gmml::Geo3DProject*)pData)->AddGeoMap(pGeo3DMap);
						}
						else if (!xmlStrcmp(prev_name, BAD_CAST"Model"))
						{
							gmmlShowProgress(strTmp.c_str());
							gmml::GeoModel* pGeoModel = new gmml::GeoModel;
							ReadGeoModel(pGeoModel, strTmp.c_str(), binary);
							((gmml::Geo3DProject*)pData)->AddGeoModel(pGeoModel);

						}
					}
				}
			}
			else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
			{
				if (!xmlStrcmp(name, BAD_CAST"Geo3DProject"))
				{
					xmlFreeTextReader(reader);
					xmlCleanupParser();
					gmmlStopProgress();
					gmmlDisplayErrorandWarningText("∂¡»°ÕÍ±œ");
					return 0;
				}
			}
			if (prev_name)
			{
				xmlFree(prev_name);
			}
			if (name)
			{
				prev_name = name;
			}
			nRet = xmlTextReaderRead(reader);
		}
		return 1;
	}
	return 0;
}