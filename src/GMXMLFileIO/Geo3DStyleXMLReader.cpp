#include "Geo3DStyleXMLReader.h"
#include "Geo3DStyle.h"
#include "AbstractTexture.h"
#include "sstream"
#include <fstream>
#include "iconv.h"
#include <string>
#include <stdlib.h>
#include <cstring>
#include "GeoSymbolizer.h"
#include "FeatureStyle.h"
#include "CoverageStyle.h"

///@brief  构造函数
Geo3DStyleXMLReader::Geo3DStyleXMLReader(void)
{
}

///@brief  析构函数
Geo3DStyleXMLReader::~Geo3DStyleXMLReader(void)
{

}

///@brief 获取GeoStyle对象
///@param 
///@author yang hongjun
///date 2013.11.09
gml::AbstractGML* Geo3DStyleXMLReader::GetGeo3DStyle()
{
	return gmmlObject;
}

///@brief 创建一个新的GeoStyle对象
///@param
///@author yang hongjun
///@date 2013.11.09
void Geo3DStyleXMLReader::CreateGeo3DStyle()
{

	gmmlObject = new gmml::Geo3DStyle;
}

///@brief 解析GeoStyle
///@param  geo3DstyleNode 指向<Style>标签
///@return 解析成功返回true，否则返回false
///@author zhaowei
///@date 2013.11.15
bool Geo3DStyleXMLReader::ReadGeo3DStyle(xmlNode* styleNode)
{
	if (!styleNode)
		return false;

	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;

	xmlChar *szKey;          //临时字符串变量
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = styleNode;
	xmlNodePtr propNodePtr = curNode;

	std::string styleID;
	std::string styleName;

	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Geo3DStyle")))
	{
		if (!gmmlObject)
			CreateGeo3DStyle();

		curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;

		if (xmlHasProp(curNode, BAD_CAST "id"))
		{
			propNodePtr = curNode;
			xmlAttrPtr attrPtr = propNodePtr->properties;
			xmlChar* szAttr = xmlGetProp(propNodePtr, BAD_CAST "id");
			char* out = 0;
			if (std::string(mXMLCoding) == "UTF-8")
				out = ConvertEnc("UTF-8", "GB2312", (char*)szAttr);

			if (out)
				styleID = std::string((char*)out);
			else
				styleID = std::string((char*)szAttr);
			curGeoStyle->setID(styleID);
			xmlFree(szAttr);

			xmlNodePtr curChildNode = curNode->xmlChildrenNode;

			// name&FeatureTypeStyle
			while (curChildNode != NULL)
			{
				//取出节点中的内容
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"name")))
				{
					//szKey = xmlNodeGetContent(curChildNode);
					szKey = xmlNodeGetContent(curChildNode);
					char* out = 0;
					if (std::string(mXMLCoding) == "UTF-8")
						out = ConvertEnc("UTF-8", "GB2312", (char*)szKey);
					std::string sr;
					if (out)
						sr = std::string((char*)out);
					else
						sr = std::string((char*)szKey);

					curGeoStyle->setName(sr);
					xmlFree(szKey);
				}
				else if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"FeatureTypeStyle")))
				{
					se::FeatureType3DStyle* featureTypeStyle = new se::FeatureType3DStyle;
					curGeoStyle->SetFeatureTypeStyle(featureTypeStyle);
					xmlNodePtr ruleNode = curChildNode->xmlChildrenNode;
					// rule
					while (ruleNode != NULL)
					{
						if ((!xmlStrcmp(ruleNode->name, (const xmlChar *)"rule")) || (!xmlStrcmp(ruleNode->name, (const xmlChar *)"Rule")))
						{
							se::FeatureStyleRule* featureStyleRule = new se::FeatureStyleRule;
							featureTypeStyle->addRule(featureStyleRule);
							xmlNodePtr AppearenceNode = ruleNode->xmlChildrenNode;
							while (AppearenceNode)
							{
								if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"Filter")))
								{
									ReadFilter(featureStyleRule, AppearenceNode);
								}
								else if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoLineSymbolizer")))
								{
									ReadSymbolizer(featureStyleRule, AppearenceNode);
								}
								else if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoPointSymbolizer")))
								{
									ReadSymbolizer(featureStyleRule, AppearenceNode);
								}
								else if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoSurfaceSymbolizer")))
								{
									ReadSymbolizer(featureStyleRule, AppearenceNode);
								}
								AppearenceNode = AppearenceNode->next;
							}
						}
						ruleNode = ruleNode->next;
					}
				}
				else if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"CoverageType3DStyle")))
				{
					se::CoverageType3DStyle *CoverageType = new se::CoverageType3DStyle;
					curGeoStyle->SetCoverageTypeStyle(CoverageType);
					xmlNodePtr ruleNode = curChildNode->xmlChildrenNode;
					// rule
					while (ruleNode != NULL)
					{
						if ((!xmlStrcmp(ruleNode->name, (const xmlChar *)"rule")) || (!xmlStrcmp(ruleNode->name, (const xmlChar *)"Rule")))
						{
							se::CoverageStyleRule* coverageStyleRule = new se::CoverageStyleRule;
							CoverageType->addRule(coverageStyleRule);
							xmlNodePtr AppearenceNode = ruleNode->xmlChildrenNode;
							while (AppearenceNode)
							{
								if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"Filter")))
								{
									ReadCoverageFilter(coverageStyleRule, AppearenceNode);
								}
								else if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoDiscreteCoverageSymbolizer")))
								{
									ReadCoverageSymbolizer(coverageStyleRule, AppearenceNode);
								}
								AppearenceNode = AppearenceNode->next;
							}
						}
						ruleNode = ruleNode->next;
					}
				}
				curChildNode = curChildNode->next;
			}
		}
	}
	return true;
}

bool Geo3DStyleXMLReader::ReadSymbolizer(se::FeatureStyleRule* rule, xmlNode* AppearenceNode)
{
	if (!gmmlObject)
		return false;

	if (!rule)
		return false;

	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;
	xmlChar *szKey;

	if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoSurfaceSymbolizer")))
	{
		se::GeofaceSymbolizer * Symbolizer = new se::GeofaceSymbolizer;
		rule->SetSymbolizer(Symbolizer);
		rule->SymbolizerType = "GeofaceSymbolizer";
		xmlNodePtr ENode = AppearenceNode->xmlChildrenNode;

		while (ENode)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"Front")))
			{
				xmlNodePtr curx3dChildNode = ENode->xmlChildrenNode;
				while (curx3dChildNode)
				{
					if ((!xmlStrcmp(curx3dChildNode->name, (const xmlChar *)"Material")))
					{
						ReadMaterial(Symbolizer, curx3dChildNode);
					}

					if ((!xmlStrcmp(curx3dChildNode->name, (const xmlChar *)"GeoReferencedTexture")))
					{
						ReadGeoReferencedTexture(Symbolizer, curx3dChildNode);
					}
					curx3dChildNode = curx3dChildNode->next;
				}
			}
			else if ((!xmlStrcmp(ENode->name, (const xmlChar *)"Back")))
			{
				xmlNodePtr curx3dChildNode = ENode->xmlChildrenNode;
				while (curx3dChildNode)
				{
					if ((!xmlStrcmp(curx3dChildNode->name, (const xmlChar *)"Material")))
					{
						ReadMaterial(Symbolizer->Back, curx3dChildNode);	/// 如果是背面的可视化参数，根据数据结构定义，
																			//存放在GeoSurfaceSymbolizer* Symbolizer的GeoSurfaceSymbolizer* Back成员变量里面
					}
					curx3dChildNode = curx3dChildNode->next;
				}
			}
			ENode = ENode->next;
		}
	}

	if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoLineSymbolizer")))
	{
		se::GeoLineSymbolizer * Symbolizer = new se::GeoLineSymbolizer;
		rule->SetSymbolizer(Symbolizer);
		rule->SymbolizerType = "LineSymbolizer";
		xmlNodePtr ENode = AppearenceNode->xmlChildrenNode;
		while (ENode)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"Stroke")))
			{
				xmlNodePtr x3dChildNode = ENode->xmlChildrenNode;
				while (x3dChildNode != NULL)
				{
					if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"SvgParameter")))
					{
						xmlNodePtr propNodePtr = x3dChildNode;
						xmlAttrPtr attrPtr = propNodePtr->properties;
						xmlChar* szAttr = xmlGetProp(propNodePtr, BAD_CAST "name");
						if ((!xmlStrcmp(szAttr, (const xmlChar *)"stroke-width")))
						{
							szKey = xmlNodeGetContent(propNodePtr);
							Symbolizer->LineWidth = atoi((char*)szKey);
							xmlFree(szKey);
						}
						xmlFree(szAttr);
					}
					x3dChildNode = x3dChildNode->next;
				}
			}
			else if ((!xmlStrcmp(ENode->name, (const xmlChar *)"Material")))
			{
				ReadMaterial(Symbolizer, ENode);
			}

			ENode = ENode->next;
		}
	}

	else if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoPointSymbolizer")))
	{
		se::GeoPointSymbolizer * Symbolizer = new se::GeoPointSymbolizer;
		rule->SetSymbolizer(Symbolizer);
		rule->SymbolizerType = "PointSymbolizer";
		xmlNodePtr ENode = AppearenceNode->xmlChildrenNode;

		while (ENode)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"Graphic")))
			{
				xmlNodePtr x3dChildNode = ENode->xmlChildrenNode;
				while (x3dChildNode != NULL)
				{
					if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"Size")))
					{
						szKey = xmlNodeGetContent(x3dChildNode);
						Symbolizer->PointSize = atoi((char*)szKey);
						xmlFree(szKey);
					}
					x3dChildNode = x3dChildNode->next;
				}
			}
			else if ((!xmlStrcmp(ENode->name, (const xmlChar *)"Material")))
			{
				ReadMaterial(Symbolizer, ENode);
			}
			ENode = ENode->next;
		}
	}
	return true;
}


bool Geo3DStyleXMLReader::ReadRasterSymbolizer(se::CoverageStyleRule* rule, xmlNode* AppearenceNode)
{
	if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"RasterSymbolizer")))
	{
		se::RasterSymbolizer * Symbolizer = new se::RasterSymbolizer;
		rule->SetSymbolizer(Symbolizer);
		rule->SymbolizerType = "RasterSymbolizer";
		xmlNodePtr ENode = AppearenceNode->xmlChildrenNode;
		while (ENode)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"ColorMap")))
			{
				xmlChar* szAttr = xmlGetProp(ENode, BAD_CAST "type");
				int type = atoi((char*)szAttr);
				Symbolizer->SetType(type);
				xmlFree(szAttr);
				ReadColorMap(Symbolizer, ENode);
			}
			ENode = ENode->next;
		}

	}
	return true;
}


bool Geo3DStyleXMLReader::ReadCoverageSymbolizer(se::CoverageStyleRule* rule, xmlNode* AppearenceNode)
{
	if (!gmmlObject)
		return false;

	if (!rule)
		return false;

	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;
	xmlChar *szKey;

	if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"GeoDiscreteCoverageSymbolizer")))
	{
		se::GeoDiscreteCoverageSymbolizer * Symbolizer = new se::GeoDiscreteCoverageSymbolizer;
		rule->SetSymbolizer(Symbolizer);
		rule->SymbolizerType = "GeoDiscreteCoverageSymbolizer";
		xmlNodePtr ENode = AppearenceNode->xmlChildrenNode;

		while (ENode)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"rule")) || (!xmlStrcmp(ENode->name, (const xmlChar *)"Rule")))
			{

				se::CoverageStyleRule* coverageStyleRule = new se::CoverageStyleRule;
				Symbolizer->addRule(coverageStyleRule);
				xmlNodePtr AppearenceNode = ENode->xmlChildrenNode;
				while (AppearenceNode)
				{
					if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"Filter")))
					{
						ReadCoverageFilter(coverageStyleRule, AppearenceNode);
					}
					else if ((!xmlStrcmp(AppearenceNode->name, (const xmlChar *)"RasterSymbolizer")))
					{
						ReadRasterSymbolizer(coverageStyleRule, AppearenceNode);
					}


					AppearenceNode = AppearenceNode->next;
				}

			}

			ENode = ENode->next;
		}
	}
	return true;
}

bool Geo3DStyleXMLReader::ReadFilter(se::FeatureStyleRule* rule, xmlNode* FilterNode)
{
	if (!rule)
		return false;

	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;
	xmlChar *szKey;          //临时字符串变量

	if ((!xmlStrcmp(FilterNode->name, (const xmlChar *)"Filter")) ||
		(!xmlStrcmp(FilterNode->name, (const xmlChar *)"ElseFilter")))
	{
		xmlNodePtr ENode = FilterNode->xmlChildrenNode;
		while (ENode != NULL)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"PropertyIsEqualTo")))
			{
				rule->FilterType = "PropertyIsEqualTo";

				xmlNodePtr fidNode = ENode->xmlChildrenNode;
				while (fidNode != NULL)
				{
					if ((!xmlStrcmp(fidNode->name, (const xmlChar *)"PropertyName")))
					{
						/*
						szKey = xmlNodeGetContent(fidNode);
						int leftc = TrimLeft((char*)szKey);
						int rightc = TrimRight((char*)szKey);
						int len = strlen((char*)szKey);
						std::string FilterName = (char*)szKey;
						FilterName = FilterName.substr(leftc,len -rightc-leftc);
						*/
						szKey = xmlNodeGetContent(fidNode);
						char* out = 0;
						if (std::string(mXMLCoding) == "UTF-8")
							out = ConvertEnc("UTF-8", "GB2312", (char*)szKey);
						std::string sr;
						if (out)
							sr = std::string((char*)out);
						else
							sr = std::string((char*)szKey);
						rule->PropertyName = sr;
						xmlFree(szKey);
					}
					else if ((!xmlStrcmp(fidNode->name, (const xmlChar *)"Literal")))
					{
						/*
						szKey = xmlNodeGetContent(fidNode);

						int leftc = TrimLeft((char*)szKey);
						int rightc = TrimRight((char*)szKey);
						int len = strlen((char*)szKey);
						std::string Properyvalue = (char*)szKey;
						Properyvalue = Properyvalue.substr(leftc,len -rightc-leftc);
						*/
						szKey = xmlNodeGetContent(fidNode);
						char* out = 0;
						if (std::string(mXMLCoding) == "UTF-8")
							out = ConvertEnc("UTF-8", "GB2312", (char*)szKey);
						std::string sr;
						if (out)
							sr = std::string((char*)out);
						else
							sr = std::string((char*)szKey);
						rule->Literal = sr;
						xmlFree(szKey);
					}
					fidNode = fidNode->next;
				}
			}
			ENode = ENode->next;
		}
		return true;
	}
	else
		return false;
}

bool Geo3DStyleXMLReader::ReadCoverageFilter(se::CoverageStyleRule* rule, xmlNode* FilterNode)
{
	if (!rule)
		return false;

	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;
	xmlChar *szKey;          //临时字符串变量

	if ((!xmlStrcmp(FilterNode->name, (const xmlChar *)"Filter")) ||
		(!xmlStrcmp(FilterNode->name, (const xmlChar *)"ElseFilter")))
	{
		xmlNodePtr ENode = FilterNode->xmlChildrenNode;
		while (ENode != NULL)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"PropertyIsEqualTo")))
			{
				rule->FilterType = "PropertyIsEqualTo";

				xmlNodePtr fidNode = ENode->xmlChildrenNode;
				while (fidNode != NULL)
				{
					if ((!xmlStrcmp(fidNode->name, (const xmlChar *)"PropertyName")))
					{
						/*
						szKey = xmlNodeGetContent(fidNode);
						int leftc = TrimLeft((char*)szKey);
						int rightc = TrimRight((char*)szKey);
						int len = strlen((char*)szKey);
						std::string FilterName = (char*)szKey;
						FilterName = FilterName.substr(leftc,len -rightc-leftc);
						*/
						szKey = xmlNodeGetContent(fidNode);
						char* out = 0;
						if (std::string(mXMLCoding) == "UTF-8")
							out = ConvertEnc("UTF-8", "GB2312", (char*)szKey);
						std::string sr;
						if (out)
							sr = std::string((char*)out);
						else
							sr = std::string((char*)szKey);
						rule->PropertyName = sr;
						xmlFree(szKey);
					}
					else if ((!xmlStrcmp(fidNode->name, (const xmlChar *)"Literal")))
					{
						/*
						szKey = xmlNodeGetContent(fidNode);

						int leftc = TrimLeft((char*)szKey);
						int rightc = TrimRight((char*)szKey);
						int len = strlen((char*)szKey);
						std::string Properyvalue = (char*)szKey;
						Properyvalue = Properyvalue.substr(leftc,len -rightc-leftc);
						*/
						szKey = xmlNodeGetContent(fidNode);
						char* out = 0;
						if (std::string(mXMLCoding) == "UTF-8")
							out = ConvertEnc("UTF-8", "GB2312", (char*)szKey);
						std::string sr;
						if (out)
							sr = std::string((char*)out);
						else
							sr = std::string((char*)szKey);
						rule->Literal = sr;
						xmlFree(szKey);
					}
					fidNode = fidNode->next;
				}
			}
			ENode = ENode->next;
		}
		return true;
	}
	else
		return false;
}

bool Geo3DStyleXMLReader::ReadGeoReferencedTexture(se::GeoSymbolizer* symbolizer, xmlNode* symbolizerNode)
{
	if (!gmmlObject)
		return false;

	if (!symbolizer)
		return false;

	xmlChar *szKey;
	se::GeofaceSymbolizer * Symbolizer = (se::GeofaceSymbolizer *) symbolizer;
	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;
	gmml::GeoReferencedTexture*tex = new gmml::GeoReferencedTexture;
	Symbolizer->SetTexture(tex);
	if ((!xmlStrcmp(symbolizerNode->name, (const xmlChar *)"GeoReferencedTexture")))
	{
		xmlNodePtr x3dChildNode = symbolizerNode->xmlChildrenNode;
		while (x3dChildNode != NULL)
		{
			if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"ImageURI")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);

				char* out = NULL;
				if (std::string(mXMLCoding) == "UTF-8")
					out = ConvertEnc("UTF-8", "GB2312", (char*)szKey);
				std::string sr;
				if (out)
					sr = std::string((char*)out);
				else
					sr = std::string((char*)szKey);
				tex->ImageURI = sr;

				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"MimeType")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);

				char* out = NULL;
				if (std::string(mXMLCoding) == "UTF-8")
					out = ConvertEnc("UTF-8", "GB2312", (char*)szKey);
				std::string sr;
				if (out)
					sr = std::string((char*)out);
				else
					sr = std::string((char*)szKey);
				tex->MimeType = sr;

				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"WrapMode")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);

				int nstrlen = strlen((char*)szKey);
				if (nstrlen)
				{
					tex->WrapMode = atoi((char*)szKey);
				}

				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"Repeat")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);

				int nstrlen = strlen((char*)szKey);
				if (nstrlen)
				{
					tex->RepeatedCount = atoi((char*)szKey);
				}

				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"BorderColor")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);
				double border[4];
				std::istringstream zhuanhuan((char*)szKey);
				zhuanhuan >> border[0] >> border[1] >> border[2] >> border[3];
				tex->BorderColor[0] = border[0];
				tex->BorderColor[1] = border[1];
				tex->BorderColor[2] = border[2];
				tex->BorderColor[3] = border[3];
				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"ReferencePoint")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);
				double border[3];
				std::istringstream zhuanhuan((char*)szKey);
				zhuanhuan >> border[0] >> border[1] >> border[2];
				tex->ReferencePoint[0] = border[0];
				tex->ReferencePoint[1] = border[1];
				tex->ReferencePoint[2] = border[2];

				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"Orientation")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);
				double border[4];
				std::istringstream zhuanhuan((char*)szKey);
				zhuanhuan >> border[0] >> border[1] >> border[2] >> border[3];
				tex->Orientation[0] = border[0];
				tex->Orientation[1] = border[1];
				tex->Orientation[2] = border[2];
				tex->Orientation[3] = border[3];
				xmlFree(szKey);


			}
			x3dChildNode = x3dChildNode->next;
		}
		return true;
	}
	else
	{
		return false;
	}

}

bool Geo3DStyleXMLReader::ReadMaterial(se::GeoSymbolizer* symbolizer, xmlNode* symbolizerNode)
{
	if (!gmmlObject)
		return false;

	if (!symbolizer)
		return false;

	xmlChar *szKey;
	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;

	if ((!xmlStrcmp(symbolizerNode->name, (const xmlChar *)"Material")))
	{
		xmlNodePtr x3dChildNode = symbolizerNode->xmlChildrenNode;
		while (x3dChildNode != NULL)
		{
			if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"DiffuseColor")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);

				int nstrlen = strlen((char*)szKey);
				if (nstrlen)
				{
					sscanf((char*)szKey, "%lf %lf %lf", &symbolizer->DiffuseColor[0], &symbolizer->DiffuseColor[1], &symbolizer->DiffuseColor[2]);
				}

				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"EmissiveColor")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);

				int nstrlen = strlen((char*)szKey);
				if (nstrlen) {

					sscanf((char*)szKey, "%lf %lf %lf", &symbolizer->EmissiveColor[0], &symbolizer->EmissiveColor[1], &symbolizer->EmissiveColor[2]);
				}
				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"SpecularColor")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);
				int nstrlen = strlen((char*)szKey);
				if (nstrlen) {

					sscanf((char*)szKey, "%lf %lf %lf", &symbolizer->SpecularColor[0], &symbolizer->SpecularColor[1], &symbolizer->SpecularColor[2]);
				}
				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"Shininess")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);

				int nstrlen = strlen((char*)szKey);
				if (nstrlen)
				{
					symbolizer->Shininess = atof((char*)szKey);
				}
				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"AmbientIntensity")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);
				int nstrlen = strlen((char*)szKey);
				if (nstrlen)
				{
					symbolizer->AmbientIntensity = atof((char*)szKey);
				}
				xmlFree(szKey);
			}
			else if ((!xmlStrcmp(x3dChildNode->name, (const xmlChar *)"Transparency")))
			{
				szKey = xmlNodeGetContent(x3dChildNode);
				int nstrlen = strlen((char*)szKey);
				if (nstrlen)
				{
					symbolizer->Transparency = atof((char*)szKey);

					if (symbolizer->Transparency == 1)
						symbolizer->Transparency = 0; //zw 2014.3
				}
				xmlFree(szKey);
			}
			x3dChildNode = x3dChildNode->next;
		}
		return true;
	}
	else
		return false;
}


bool Geo3DStyleXMLReader::ReadColorMap(se::RasterSymbolizer* symbolizer, xmlNode* symbolizerNode)
{
	if (!gmmlObject)
		return false;

	if (!symbolizer)
		return false;


	gmml::Geo3DStyle* curGeoStyle = (gmml::Geo3DStyle*)gmmlObject;
	if ((!xmlStrcmp(symbolizerNode->name, (const xmlChar *)"ColorMap")))
	{
		std::vector<std::string> color;
		std::vector<double> aquality;
		std::vector<double> opacity;
		std::vector<std::string> labels;
		xmlNodePtr ENode = symbolizerNode->xmlChildrenNode;
		while (ENode != NULL)
		{
			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"Geometry")))
			{
				xmlChar* szAttr = xmlNodeGetContent(ENode);
				char* out = NULL;
				if (std::string(mXMLCoding) == "UTF-8")
					out = ConvertEnc("UTF-8", "GB2312", (char*)szAttr);
				std::string sr;
				if (out)
					sr = std::string((char*)out);
				else
					sr = std::string((char*)szAttr);

				symbolizer->SetGeometry(sr);
			}


			if ((!xmlStrcmp(ENode->name, (const xmlChar *)"ColorMapEntry")))
			{
				xmlChar* szAttr = xmlGetProp(ENode, BAD_CAST "color");
				char* out = NULL;
				if (std::string(mXMLCoding) == "UTF-8")
					out = ConvertEnc("UTF-8", "GB2312", (char*)szAttr);
				std::string sr;
				if (out)
					sr = std::string((char*)out);
				else
					sr = std::string((char*)szAttr);
				color.push_back(sr);



				xmlChar*szAttr1 = xmlGetProp(ENode, BAD_CAST "label");
				char* out1 = NULL;
				if (std::string(mXMLCoding) == "UTF-8")
					out1 = ConvertEnc("UTF-8", "GB2312", (char*)szAttr1);
				std::string sr1;
				if (out1)
					sr1 = std::string((char*)out1);
				else
					sr1 = std::string((char*)szAttr1);
				labels.push_back(sr1);
				xmlFree(szAttr1);

				xmlChar* szAttr2 = xmlGetProp(ENode, BAD_CAST "quantity");
				int nstrlen = strlen((char*)szAttr2);
				if (nstrlen)
				{
					double shu = atof((char*)szAttr2);
					aquality.push_back(shu);
				}
				xmlFree(szAttr2);

				xmlChar* szAttr3 = xmlGetProp(ENode, BAD_CAST "opacity");
				nstrlen = strlen((char*)szAttr3);
				if (nstrlen)
				{
					double shu = atof((char*)szAttr3);
					opacity.push_back(shu);
				}
				xmlFree(szAttr3);

			}

			ENode = ENode->next;
		}

		//symbolizer->CreateMap(color.size());
		for (int shumu = 0; shumu < color.size(); shumu++)
		{
			symbolizer->AddMapEntry(aquality[shumu], color[shumu], opacity[shumu], labels[shumu]);
		}

		return true;

	}

	else
	{
		return false;
	}

}
