//#include "StdAfx.h"
#include "FieldReader.h"


int ReadField(AttributeValue&value,xmlTextReaderPtr reader)
{
	int nRet = 1;
	xmlChar* name = NULL;
	while (nRet == 1) 		
	{
		name = xmlTextReaderName(reader);

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"swe:field")||
				!xmlStrcmp(name,BAD_CAST"Field"))
			{
				char* out = ReadAttrAndConvert(reader,"name");
				if (!out)
				{
					out = ReadAttrAndConvert(reader,"Name");
				}
				value.fieldName = out;
			}
			else if (!xmlStrcmp(name,BAD_CAST"swe:Text")||!xmlStrcmp(name,BAD_CAST"Text"))
			{
				value.fieldType = "Text";
			}
			else if (!xmlStrcmp(name,BAD_CAST"swe:Count")||!xmlStrcmp(name,BAD_CAST"Count"))
			{
				value.fieldType = "Count";
			}
			else if (!xmlStrcmp(name,BAD_CAST"swe:Double")||!xmlStrcmp(name,BAD_CAST"Double"))
			{
				value.fieldType = "Double";
			}
			else if (!xmlStrcmp(name,BAD_CAST"swe:value"))
			{
				value.fieldValue =(std::string) (ReadValueAndConvert(reader));
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"swe:field")||
				!xmlStrcmp(name,BAD_CAST"Field"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
}

