//#include "StdAfx.h"
#include "Geo3DLayerReader.h"


#include "Geo3DLayer.h"
#include "FeatureClass.h"
#include "Geo3DStyle.h"
#include "Geo3DStyleReader.h"


#include <string>

using gmml::Geo3DLayer;
using std::string;

int ReadGeo3DLayer(void* pData,xmlTextReaderPtr reader)
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
			if (!xmlStrcmp(name,BAD_CAST"id"))
			{
				char* out = ReadAttrAndConvert(reader,"id");//check 
				if (out)
				{
					((Geo3DLayer*)pData)->setID(out);
				}


			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:name"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					((Geo3DLayer*)pData)->setName(out);
				}
				xmlFree(value);

			}
			else if (!xmlStrcmp(name,BAD_CAST"FeatureClass"))
			{
				char* out = ReadAttrAndConvert(reader,"xlink:type");
				if (out)
				{

				}
				string strOut = ReadAttrAndConvert(reader,"xlink:href");
				if (strOut.size())
				{
					if(strOut.find("#")>=0)                   ///@因为GeologicFeatureClass的id的开头是带有#的
						strOut = strOut.substr(strOut.find("#")+1,strOut.size()-1);

					gmml::GeologicFeatureClass *fl = new gmml::GeologicFeatureClass;
					fl->setID(strOut);
					fl->setContainer((Geo3DLayer*)pData);

					((Geo3DLayer*)pData)->SetFeatureClass(fl); 
				}

			}

			else if (!xmlStrcmp(name,BAD_CAST"Geo3DStyle"))
			{
				gmml::Geo3DStyle* pGeo3DStyle = new gmml::Geo3DStyle();
				ReadGeo3DStyle(pGeo3DStyle,reader);
				((Geo3DLayer*)pData)->SetStyle(pGeo3DStyle);
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Layer"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
	
	

	return -1;
}