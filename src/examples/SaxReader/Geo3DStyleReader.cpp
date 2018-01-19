//#include "StdAfx.h"
#include "Geo3DStyleReader.h"
#include "sstream"
#include "Geo3DStyle.h"
#include "GeoSymbolizer.h"
#include "AbstractTexture.h"
#include "CoverageStyle.h"
using gmml::Geo3DStyle;

int ReadGeo3DStyle(void* pData,xmlTextReaderPtr reader)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	std::string pre_name;//记录上一个标签名称
	se::FeatureType3DStyle* featureTypeStyle = NULL;
	se::FeatureStyleRule* featureStyleRule = NULL;

	se::CoverageType3DStyle * coverageTypeStyle = NULL;
	se::CoverageStyleRule* coverageStyleRule = NULL;

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
			if (!xmlStrcmp(name,BAD_CAST"Geo3DStyle"))
			{
				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					((Geo3DStyle*)pData)->setID(out);
				}


			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:name"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					((Geo3DStyle*)pData)->setName(out);
				}
				xmlFree(value);

			}
			else if (!xmlStrcmp(name,BAD_CAST"se:CoverageType3DStyle"))
			{
				pre_name="se:CoverageType3DStyle";
				coverageTypeStyle = new se::CoverageType3DStyle;						
				((Geo3DStyle*)pData)->SetCoverageTypeStyle(coverageTypeStyle);	
			}

			else if (!xmlStrcmp(name,BAD_CAST"se:FeatureTypeStyle"))
			{
				pre_name="se:FeatureTypeStyle";
				featureTypeStyle = new se::FeatureType3DStyle;						
				((Geo3DStyle*)pData)->SetFeatureTypeStyle(featureTypeStyle);	
			}
			else if (!xmlStrcmp(name,BAD_CAST"se:Rule")||
				!xmlStrcmp(name,BAD_CAST"se:rule"))
			{
				if(pre_name=="se:FeatureTypeStyle")
				{
					featureStyleRule = new se::FeatureStyleRule;
					if (featureTypeStyle)
					{
						featureTypeStyle->addRule(featureStyleRule);
					}
				}
				

				if(pre_name=="se:CoverageType3DStyle")
				{
					coverageStyleRule = new se::CoverageStyleRule;
					if (coverageTypeStyle)
					{
						coverageTypeStyle->addRule(coverageStyleRule);
					}
				}
				

			}

			else if (!xmlStrcmp(name,BAD_CAST"ogc:Filter")
				||!xmlStrcmp(name,BAD_CAST"ogc:ElseFilter"))
			{
				if(pre_name=="se:FeatureTypeStyle")
				{
					ReadFilter(featureStyleRule,reader);
				}

				if(pre_name=="se:CoverageType3DStyle")
				{
					ReadCovergeFilter(coverageStyleRule,reader);
				}
				
				
			}
			else if((!xmlStrcmp(name, (const xmlChar *)"GeoLineSymbolizer"))) 
			{
				ReadSymbolizer(featureStyleRule,reader);
			}
			else if((!xmlStrcmp(name, (const xmlChar *)"GeoPointSymbolizer"))) 
			{
				ReadSymbolizer(featureStyleRule,reader,"GeoPointSymbolizer");
			}
			else if((!xmlStrcmp(name, (const xmlChar *)"GeoSurfaceSymbolizer")))
			{
				ReadSymbolizer(featureStyleRule,reader);
			}
			else if((!xmlStrcmp(name, (const xmlChar *)"GeoDiscreteCoverageSymbolizer")))
			{
				ReadCovergeSymbolizer(coverageStyleRule,reader);
			}

			/*else if((!xmlStrcmp(name, (const xmlChar *)"RasterSymbolizer")))
			{
				ReadCovergeSymbolizer(coverageStyleRule,reader);
			}*/

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Geo3DStyle"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}



	return -1;
}


int ReadFilter(void* pData,xmlTextReaderPtr reader)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	se::FeatureStyleRule* rule = (se::FeatureStyleRule*)pData;


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
			if (!xmlStrcmp(name,BAD_CAST"ogc:PropertyIsEqualTo"))
			{
				rule->FilterType = "PropertyIsEqualTo";
			}
			else if (!xmlStrcmp(name,BAD_CAST"ogc:PropertyName"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					rule->PropertyName = out;
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"ogc:Literal"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					rule->Literal = out;
				}
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"ogc:Filter")||!xmlStrcmp(name,BAD_CAST"ogc:ElseFilter"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadCovergeFilter(void* pData,xmlTextReaderPtr reader)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	se::CoverageStyleRule* rule = (se::CoverageStyleRule*)pData;


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
			if (!xmlStrcmp(name,BAD_CAST"ogc:PropertyIsEqualTo"))
			{
				rule->FilterType = "PropertyIsEqualTo";
			}
			else if (!xmlStrcmp(name,BAD_CAST"ogc:PropertyName"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					rule->PropertyName = out;
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"ogc:Literal"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					rule->Literal = out;
				}
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"ogc:Filter")||!xmlStrcmp(name,BAD_CAST"ogc:ElseFilter"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
}



int ReadSymbolizer(void* pData,xmlTextReaderPtr reader,const char* pSymbolizer)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	se::FeatureStyleRule* rule = (se::FeatureStyleRule*)pData;

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
			if (!xmlStrcmp(name,BAD_CAST"GeoSurfaceSymbolizer"))
			{
				se:: GeofaceSymbolizer *Symbolizer = new se:: GeofaceSymbolizer;
				rule->SetSymbolizer(Symbolizer);
				rule->SymbolizerType = "GeofaceSymbolizer";

				return ReadGeofaceSymbolizer(Symbolizer,reader);
			}
			else if (!xmlStrcmp(name,BAD_CAST"GeoLineSymbolizer"))
			{
				se::GeoLineSymbolizer*LineSymbolizer = new se::GeoLineSymbolizer;
				rule->SetSymbolizer(LineSymbolizer);
				rule->SymbolizerType = "LineSymbolizer";

				return ReadGeoLineSymbolizer(LineSymbolizer,reader);
			}
			else if (!xmlStrcmp(name,BAD_CAST"GeoPointSymbolizer"))
			{
				se::GeoPointSymbolizer *PointSymbolizer = new se::GeoPointSymbolizer;
				rule->SetSymbolizer(PointSymbolizer);
				rule->SymbolizerType = "PointSymbolizer";

				return ReadPointSymbolizer(PointSymbolizer,reader);
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoLineSymbolizer")||
				!xmlStrcmp(name,BAD_CAST"GeoSurfaceSymbolizer")||
				!xmlStrcmp(name,BAD_CAST"GeoPointSymbolizer"))
			{
				return 0;
			}

		}


		if (name)
		{
			xmlFree(name);
			name = NULL;
		}
		nRet = xmlTextReaderRead(reader);
	}
}


int ReadCovergeSymbolizer(void* pData,xmlTextReaderPtr reader,const char* pSymbolizer)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	se::CoverageStyleRule* rule = (se::CoverageStyleRule*)pData;
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
			if (!xmlStrcmp(name,BAD_CAST"GeoDiscreteCoverageSymbolizer"))
			{
				se:: GeoDiscreteCoverageSymbolizer * Symbolizer = new se:: GeoDiscreteCoverageSymbolizer;
				rule->SetSymbolizer(Symbolizer);
				rule->SymbolizerType = "GeoDiscreteCoverageSymbolizer";

				ReadGeoDiscreteCoverageSymbolizer(Symbolizer,reader);
			}
			
			
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoDiscreteCoverageSymbolizer"))
			{
				return 0;
			}

		}


		if (name)
		{
			xmlFree(name);
			name = NULL;
		}
		nRet = xmlTextReaderRead(reader);
	}
}

int ReadGeoDiscreteCoverageSymbolizer(void* pData,xmlTextReaderPtr reader)
{
	se::CoverageStyleRule*coverageStyleRule;
	xmlChar* name = NULL,*value = NULL;
	int nRet = 1;
	 se:: GeoDiscreteCoverageSymbolizer* pGeoGeoDiscreteCoverageSymbolizer = (se:: GeoDiscreteCoverageSymbolizer*)pData;


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
			if (!xmlStrcmp(name,BAD_CAST"se:Rule")||
				!xmlStrcmp(name,BAD_CAST"se:rule"))
			{
				coverageStyleRule = new se::CoverageStyleRule;
				if (pGeoGeoDiscreteCoverageSymbolizer)
				{
					pGeoGeoDiscreteCoverageSymbolizer->addRule(coverageStyleRule);
				}
			}


			else if (!xmlStrcmp(name,BAD_CAST"ogc:Filter")
				||!xmlStrcmp(name,BAD_CAST"ogc:ElseFilter"))
			{
			
					ReadCovergeFilter(coverageStyleRule,reader);
			}

			else if((!xmlStrcmp(name, (const xmlChar *)"se:RasterSymbolizer")))
			{
				ReadRasterSymbolizer(coverageStyleRule,reader);
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"se:Rule")||
				!xmlStrcmp(name,BAD_CAST"se:rule"))
			{
				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}		
}

int ReadRasterSymbolizer(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,*value = NULL;
	int nRet = 1;
	se::CoverageStyleRule* rule = (se::CoverageStyleRule*)pData;
	
	 std::vector<std::string> color;
	 std::vector<double> aquality;
	 std::vector<double> opacity;
	 std::vector<std::string> labels;
	 se:: RasterSymbolizer * pGeoRasterSymbolizer = new se:: RasterSymbolizer;
	 rule->SetSymbolizer(pGeoRasterSymbolizer);
	 rule->SymbolizerType = "RasterSymbolizer";

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
			if (!xmlStrcmp(name,BAD_CAST"ColorMap"))
			{
				if (xmlTextReaderAttributeCount(reader)>0)
				{
					xmlChar* value = xmlTextReaderGetAttribute(reader,(xmlChar*)"type");
					if (value)
					{
						int type=atoi((char*)value);
						pGeoRasterSymbolizer->SetType(type);
						xmlFree(value);
					}
				}
				
			}

			else if (!xmlStrcmp(name,BAD_CAST"se:Geometry"))
			{
				
				char* valuee = ReadValueAndConvert(reader);
				if (valuee)
				{
					std::string abc=std::string(valuee);
					pGeoRasterSymbolizer->SetGeometry(abc);

				}

			}
			else if (!xmlStrcmp(name,BAD_CAST"ColorMapEntry"))
			{

				if (xmlTextReaderAttributeCount(reader)>0)
				{
					char* value1 = ReadAttrAndConvert(reader,"color");
					if (value1)
					{
						color.push_back(value1);
						
					}

					xmlChar* value2 = xmlTextReaderGetAttribute(reader,(xmlChar*)"quantity");
					if (value2)
					{
						double quantities=atof((char*)value2);
						aquality.push_back(quantities);
						xmlFree(value2);
					}

					char* value3 = ReadAttrAndConvert(reader,"label");
					if (value3)
					{
						labels.push_back(value3);
					}

					xmlChar* value4 = xmlTextReaderGetAttribute(reader,(xmlChar*)"opacity");
					if (value4)
					{
						double opac=atof((char*)value4);
						opacity.push_back(opac);
						xmlFree(value4);
					}

				
				}
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"se:RasterSymbolizer"))
			{
				//pGeoRasterSymbolizer->CreateMap(color.size());
				for(int shumu=0;shumu<color.size();shumu++)
				{
					pGeoRasterSymbolizer->AddMapEntry(aquality[shumu],color[shumu],opacity[shumu],labels[shumu]);
				}


				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}		
}


int ReadGeofaceSymbolizer(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,*pre_name = NULL;
	int nRet = 1;
	se::GeofaceSymbolizer* pGeofaceSymbolizer = (se::GeofaceSymbolizer*)pData;


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
			if (!xmlStrcmp(name,BAD_CAST"Material"))
			{
				if (!xmlStrcmp(pre_name,BAD_CAST"Front"))
				{
					ReadMaterial(pGeofaceSymbolizer,reader);
				}
				else if (!xmlStrcmp(pre_name,BAD_CAST"Back"))
				{
					ReadMaterial(pGeofaceSymbolizer->Back,reader);
				}
			}

			if (!xmlStrcmp(name,BAD_CAST"GeoReferencedTexture"))
			{
				
					ReadGeoReferencedTexture(pGeofaceSymbolizer,reader);
				
				/*else if (!xmlStrcmp(pre_name,BAD_CAST"Back"))
				{
					ReadGeoReferencedTexture(pGeofaceSymbolizer,reader);
				}*/
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoSurfaceSymbolizer"))
			{
				return 0;
			}
		}

		if (pre_name)
		{
			xmlFree(pre_name);
			pre_name = NULL;
		}

		if (name)
		{
			pre_name = name;
		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadGeoLineSymbolizer(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,*value = NULL;
	int nRet = 1;
	se::GeoLineSymbolizer* pLineSymbolizer = (se::GeoLineSymbolizer*)pData;


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
			if (!xmlStrcmp(name,BAD_CAST"se:SvgParameter"))
			{
				char* szAttr = ReadAttrAndConvert(reader,"name");
				if (szAttr&&(!strcmp(szAttr,"stroke-width"))) 
				{
					char* out= (char*)xmlTextReaderReadString(reader);
					pLineSymbolizer->LineWidth = atoi(out);						
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"Material"))
			{
				ReadMaterial(pLineSymbolizer,reader);
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoLineSymbolizer"))
			{
				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}	
}

int ReadPointSymbolizer(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,*value = NULL;
	int nRet = 1;
	se::GeoPointSymbolizer* pGeoPointSymbolizer = (se::GeoPointSymbolizer*)pData;


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
			if (!xmlStrcmp(name,BAD_CAST"se:Size"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					if (pGeoPointSymbolizer)
					{
						pGeoPointSymbolizer->PointSize = atoi((char*)value);
					}
					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"Material"))
			{
				ReadMaterial(pGeoPointSymbolizer,reader);
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoPointSymbolizer"))
			{
				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}		
}

int ReadMaterial(void* pData,xmlTextReaderPtr reader)
{
	se::GeoSymbolizer* symbolizer = (se::GeoSymbolizer*)pData;

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
			if (!xmlStrcmp(name,BAD_CAST"DiffuseColor"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					int nstrlen = strlen((char*)value);
					if(nstrlen)
					{	 
						sscanf(((char*)value),"%lf %lf %lf",&symbolizer->DiffuseColor[0],&symbolizer->DiffuseColor[1],&symbolizer->DiffuseColor[2]);
					}

					xmlFree(value);
				}


			}
			else if (!xmlStrcmp(name,BAD_CAST"EmissiveColor"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					int nstrlen = strlen((char*)value);
					if(nstrlen)
					{	 
						sscanf(((char*)value),"%lf %lf %lf",&symbolizer->EmissiveColor[0],&symbolizer->EmissiveColor[1],&symbolizer->EmissiveColor[2]);
					}

					xmlFree(value);
				}

			}
			else if (!xmlStrcmp(name,BAD_CAST"SpecularColor"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					int nstrlen = strlen((char*)value);
					if(nstrlen)
					{	 
						sscanf(((char*)value),"%lf %lf %lf",&symbolizer->SpecularColor[0],&symbolizer->SpecularColor[1],&symbolizer->SpecularColor[2]);
					}

					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"Shininess"))
			{

				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{

					int nstrlen = strlen((char*)value);
					if(nstrlen)
					{	 
						symbolizer->Shininess = atof((char*)value);
					}

					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"AmbientIntensity"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					int nstrlen = strlen((char*)value);
					if(nstrlen)
					{	 
						symbolizer->AmbientIntensity = atof((char*)value);
					}

					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"Transparency"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					int nstrlen = strlen((char*)value);
					if(nstrlen)
					{	 
						symbolizer->Transparency = atof((char*)value);

						if(symbolizer->Transparency == 1)
							symbolizer->Transparency = 0; //zw 2014.3
					}

					xmlFree(value);
				}

			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Material"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadGeoReferencedTexture(void* pData,xmlTextReaderPtr reader)
{
	se::GeofaceSymbolizer* symbolizer = (se::GeofaceSymbolizer*)pData;
	gmml::GeoReferencedTexture*tex=new gmml::GeoReferencedTexture;
	symbolizer->SetTexture(tex);

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
			if (!xmlStrcmp(name,BAD_CAST"ImageURI"))
			{
				char* out = ReadValueAndConvert(reader);
			
				tex->ImageURI=out;
			}

			else if (!xmlStrcmp(name,BAD_CAST"MimeType"))
			{
				char* out = ReadValueAndConvert(reader);

				tex->MimeType=out;

			}
			else if (!xmlStrcmp(name,BAD_CAST"WrapMode"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					int nstrlen = strlen((char*)value);
					if(nstrlen)
					{	 
						tex->WrapMode = atoi((char*)value);
					}

					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"BorderColor"))
			{

				xmlChar* value = xmlTextReaderReadString(reader);
				
				double border[4];
				std::istringstream zhuanhuan((char*)value);
				zhuanhuan>>border[0]>>border[1]>>border[2]>>border[3];
				tex->BorderColor[0]=border[0];
				tex->BorderColor[1]=border[1];
				tex->BorderColor[2]=border[2];
				tex->BorderColor[3]=border[3];
				xmlFree(value);	
			}
			else if (!xmlStrcmp(name,BAD_CAST"ReferencePoint"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				
				double border[3];
				std::istringstream zhuanhuan((char*)value);
				zhuanhuan>>border[0]>>border[1]>>border[2];
				tex->ReferencePoint[0]=border[0];
				tex->ReferencePoint[1]=border[1];
				tex->ReferencePoint[2]=border[2];

					xmlFree(value);
			}
			else if (!xmlStrcmp(name,BAD_CAST"Orientation"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				
				double border[4];
				std::istringstream zhuanhuan((char*)value);
				zhuanhuan>>border[0]>>border[1]>>border[2]>>border[3];
				tex->Orientation[0]=border[0];
				tex->Orientation[1]=border[1];
				tex->Orientation[2]=border[2];
				tex->Orientation[3]=border[3];
				xmlFree(value);

			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoReferencedTexture"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
}