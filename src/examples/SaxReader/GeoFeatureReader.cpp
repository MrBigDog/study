//#include "StdAfx.h"
#include "GeoFeatureReader.h"

#include "GeologicFeature.h"
#include "FieldReader.h"
#include "MappedFeature.h"
#include "ShapeReader.h"
#include "ShapePropertyReader.h"

using gmml::GeologicFeature;
using gmml::MappedFeature;


int ReadGeoFeature(void* pData, xmlTextReaderPtr reader, int binary)
{
	int nRet = 1;
	xmlChar* name = NULL, *value = NULL;
	vtkObject* pVtkObj = NULL;
	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);


		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"GeoFeature"))
			{
				char* out = ReadAttrAndConvert(reader, "gml:id");
				if (out)
				{
					((GeologicFeature*)pData)->setID(out);
				}

			}
			else if (!xmlStrcmp(name, BAD_CAST"gml:name"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					((GeologicFeature*)pData)->setName(out);
				}

			}
			else if (!xmlStrcmp(name, BAD_CAST"Field"))
			{
				AttributeValue value;
				ReadField(value, reader);
				((GeologicFeature*)pData)->AddAttributeValue(value);
			}
			else if (!xmlStrcmp(name, BAD_CAST"Geometry"))
			{
				MappedFeature* pGeometry = new MappedFeature();
				/*vtkObject**/ pVtkObj = ReadShape(pGeometry, reader, binary);
				pGeometry->SetShape(pVtkObj);

				((GeologicFeature*)pData)->AddMappedFeature(pGeometry);
			}
			else if (!xmlStrcmp(name, BAD_CAST"ShapeProperty") ||
				!xmlStrcmp(name, BAD_CAST"PropertyModels") ||
				!xmlStrcmp(name, BAD_CAST"Propertymodels"))
			{
				ReadShapeProperty(pVtkObj, reader);
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"GeoFeature"))
			{
				return 0;
			}

		}
		xmlFree(name);
		nRet = xmlTextReaderRead(reader);
	}
}