#include "Geo3DStyleXMLWriter.h"
#include "Geo3DStyle.h"
#include "GeoSymbolizer.h"
#include <fstream>
#include "AbstractTexture.h"
#include "CoverageStyle.h"
//#include "GeoDiscreteCoverageSymbolizer.h"


namespace se
{
	class FeatureType3DStyle;
	class FeatureStyleRule;
	class GeoSymbolizer;

	class GeoPointSymbolizer;
	class GeofaceSymbolizer;
	class GeoLineSymbolizer;
}

namespace gmml
{
	class Geo3DStyle;
}

Geo3DStyleXMLWriter::Geo3DStyleXMLWriter(void)
{
}


Geo3DStyleXMLWriter::~Geo3DStyleXMLWriter(void)
{
}

void Geo3DStyleXMLWriter::WriteStyle(gmml::Geo3DStyle* style, std::ofstream& xml_stream)
{
	std::string xml_node_name = "Geo3DStyle";
	xml_stream << "<" << xml_node_name << " gml:id=\"" << style->getID() << "\">" << std::endl;
	xml_stream << "<gml:name>" << style->getName() << "</gml:name>" << std::endl;

	xml_stream << "<se:FeatureTypeStyle>" << std::endl;

	for (int i = 0; i < style->GetFeatureTypeStyleCount(); i++)
	{
		se::FeatureType3DStyle* type_style = style->GetFeatureTypeStyle(i);

		for (int j = 0; j < type_style->getRuleCount(); j++)
		{
			xml_stream << "<se:Rule>" << std::endl;

			se::FeatureStyleRule* rule = type_style->getRule(j);
			if (rule)
			{
				xml_stream << "<ogc:Filter>" << std::endl;
				xml_stream << "<ogc:PropertyIsEqualTo>" << std::endl;
				xml_stream << "<ogc:PropertyName>" << rule->PropertyName;
				xml_stream << "</ogc:PropertyName>" << std::endl;
				xml_stream << "<ogc:Literal>" << rule->Literal;
				xml_stream << "</ogc:Literal>" << std::endl;

				xml_stream << "</ogc:PropertyIsEqualTo>" << std::endl;
				xml_stream << "</ogc:Filter>" << std::endl;
			}

			WirteSymbolizer(rule, xml_stream);

			xml_stream << "</se:Rule>" << std::endl;
		}
	}

	xml_stream << "</se:FeatureTypeStyle>" << std::endl;

	xml_stream << "<se:CoverageType3DStyle>" << std::endl;

	for (int i = 0; i < style->GetCoverageTypeStyleCount(); i++)
	{
		se::CoverageType3DStyle* type_style = style->GetCoverageTypeStyle(i);

		for (int j = 0; j < type_style->getRuleCount(); j++)
		{
			xml_stream << "<se:Rule>" << std::endl;

			se::CoverageStyleRule* rule = type_style->getRule(j);
			if (rule)
			{
				xml_stream << "<ogc:Filter>" << std::endl;
				xml_stream << "<ogc:PropertyIsEqualTo>" << std::endl;
				xml_stream << "<ogc:PropertyName>" << rule->PropertyName;
				xml_stream << "</ogc:PropertyName>" << std::endl;
				xml_stream << "<ogc:Literal>" << rule->Literal;
				xml_stream << "</ogc:Literal>" << std::endl;

				xml_stream << "</ogc:PropertyIsEqualTo>" << std::endl;
				xml_stream << "</ogc:Filter>" << std::endl;
			}

			WirteCoverageSymbolizer(rule, xml_stream);

			xml_stream << "</se:Rule>" << std::endl;
		}
	}

	xml_stream << "</se:CoverageType3DStyle>" << std::endl;


	xml_stream << "</" << xml_node_name << ">" << std::endl;
}

void Geo3DStyleXMLWriter::WirteCoverageSymbolizer(se::CoverageStyleRule* rule, std::ofstream& xml_stream)
{

	std::string xml_node_name;



	xml_node_name = "GeoDiscreteCoverageSymbolizer";
	if (rule->GetSymbolizer() == NULL)
	{
		return;
	}
	xml_stream << "<" << xml_node_name << ">" << std::endl;

	if (rule->GetSymbolizer()->GetClassName() != std::string("GeoDiscreteCoverageSymbolizer"))
		return;

	se::GeoDiscreteCoverageSymbolizer* CoverageSymbolizer = (se::GeoDiscreteCoverageSymbolizer*)rule->GetSymbolizer();

	for (int rulecount = 0; rulecount < CoverageSymbolizer->getRuleCount(); rulecount++)
	{
		se::CoverageStyleRule* RasterRule = CoverageSymbolizer->getRule(rulecount);

		xml_stream << "<se:Rule>" << std::endl;


		if (RasterRule)
		{
			xml_stream << "<ogc:Filter>" << std::endl;
			xml_stream << "<ogc:PropertyIsEqualTo>" << std::endl;
			xml_stream << "<ogc:PropertyName>" << RasterRule->PropertyName;
			xml_stream << "</ogc:PropertyName>" << std::endl;
			xml_stream << "<ogc:Literal>" << RasterRule->Literal;
			xml_stream << "</ogc:Literal>" << std::endl;

			xml_stream << "</ogc:PropertyIsEqualTo>" << std::endl;
			xml_stream << "</ogc:Filter>" << std::endl;
		}

		if (RasterRule->GetSymbolizer()->GetClassName() == std::string("RasterSymbolizer"))
		{
			std::string r_node_name = "se:RasterSymbolizer";
			xml_stream << "<" << r_node_name << ">" << std::endl;
			xml_stream << "<ColorMap type=\"" << ((se::RasterSymbolizer*)(RasterRule->GetSymbolizer()))->GetType() << "\">" << std::endl;

			xml_stream << "<se:Geometry>";
			xml_stream << ((se::RasterSymbolizer*)(RasterRule->GetSymbolizer()))->GetGeometry();
			xml_stream << "</se:Geometry>" << std::endl;
			int num = ((se::RasterSymbolizer*)(RasterRule->GetSymbolizer()))->GetMapEntryCount();
			double Quanlity;
			std::string  ColorRef;
			double  Opacity;
			std::string Label;
			for (int xx = 0; xx < num; xx++)
			{
				((se::RasterSymbolizer*)(RasterRule->GetSymbolizer()))->GetMapEntry(xx, Quanlity, ColorRef, Opacity, Label);

				xml_stream << "<ColorMapEntry color=\"" << ColorRef << "\" quantity=\"" << Quanlity << "\" label=\"" << Label << "\" opacity=\"" << Opacity << "\"" << "/" << ">" << std::endl;
			}

			xml_stream << "</ColorMap>" << std::endl;

			xml_stream << "</" << r_node_name << ">" << std::endl;
		}

		xml_stream << "</se:Rule>" << std::endl;

	}

	xml_stream << "</" << xml_node_name << ">" << std::endl;
}


void Geo3DStyleXMLWriter::WirteSymbolizer(se::FeatureStyleRule* rule, std::ofstream& xml_stream)
{

	std::string xml_node_name;

	if (rule->SymbolizerType == "PointSymbolizer")
		xml_node_name = "GeoPointSymbolizer";

	else if (rule->SymbolizerType == "LineSymbolizer")
		xml_node_name = "GeoLineSymbolizer";

	else if (rule->SymbolizerType == "GeofaceSymbolizer")
		xml_node_name = "GeoSurfaceSymbolizer";
	if (rule->GetSymbolizer() == NULL)
	{
		return;
	}
	xml_stream << "<" << xml_node_name << ">" << std::endl;

	if (rule->SymbolizerType == "PointSymbolizer")
	{
		xml_stream << "<se:Graphic>" << std::endl;

		xml_stream << "<se:Size>" << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->PointSize;
		xml_stream << "</se:Size>" << std::endl;

		xml_stream << "</se:Graphic>" << std::endl;

		xml_stream << "<Material>" << std::endl;

		xml_stream << "<AmbientIntensity>" << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->AmbientIntensity;
		xml_stream << "</AmbientIntensity>" << std::endl;

		xml_stream << "<Shininess>" << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->Shininess;
		xml_stream << "</Shininess>" << std::endl;

		xml_stream << "<DiffuseColor>" << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[0] << " ";
		xml_stream << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[1] << " ";
		xml_stream << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[2];
		xml_stream << "</DiffuseColor>" << std::endl;

		xml_stream << "<EmissiveColor>" << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[0] << " ";
		xml_stream << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[1] << " ";
		xml_stream << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[2];
		xml_stream << "</EmissiveColor>" << std::endl;

		xml_stream << "<SpecularColor>" << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[0] << " ";
		xml_stream << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[1] << " ";
		xml_stream << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[2];
		xml_stream << "</SpecularColor>" << std::endl;

		if (((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->Transparency == 0)
		{
			double transparency_tmp = 1;
			xml_stream << "<Transparency>" << transparency_tmp;
		}
		else
			xml_stream << "<Transparency>" << ((se::GeoPointSymbolizer*)(rule->GetSymbolizer()))->Transparency;
		xml_stream << "</Transparency>" << std::endl;

		xml_stream << "</Material>" << std::endl;
	}

	if (rule->SymbolizerType == "LineSymbolizer")
	{
		xml_stream << "<se:Stroke>" << std::endl;

		xml_stream << "<se:SvgParameter name=\"stroke-width\">" << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->LineWidth;
		xml_stream << "</se:SvgParameter>" << std::endl;

		xml_stream << "</se:Stroke>" << std::endl;

		xml_stream << "<Material>" << std::endl;

		xml_stream << "<AmbientIntensity>" << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->AmbientIntensity;
		xml_stream << "</AmbientIntensity>" << std::endl;

		xml_stream << "<Shininess>" << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->Shininess;
		xml_stream << "</Shininess>" << std::endl;

		xml_stream << "<DiffuseColor>" << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[0] << " ";
		xml_stream << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[1] << " ";
		xml_stream << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[2];
		xml_stream << "</DiffuseColor>" << std::endl;

		xml_stream << "<EmissiveColor>" << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[0] << " ";
		xml_stream << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[1] << " ";
		xml_stream << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[2];
		xml_stream << "</EmissiveColor>" << std::endl;

		xml_stream << "<SpecularColor>" << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[0] << " ";
		xml_stream << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[1] << " ";
		xml_stream << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[2];
		xml_stream << "</SpecularColor>" << std::endl;

		if (((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->Transparency == 0)
		{
			double transparency_tmp = 1;
			xml_stream << "<Transparency>" << transparency_tmp;
		}
		else
			xml_stream << "<Transparency>" << ((se::GeoLineSymbolizer*)(rule->GetSymbolizer()))->Transparency;
		xml_stream << "</Transparency>" << std::endl;

		xml_stream << "</Material>" << std::endl;
	}

	if (rule->SymbolizerType == "GeofaceSymbolizer")
	{
		xml_stream << "<Front>" << std::endl;

		xml_stream << "<Material>" << std::endl;

		xml_stream << "<AmbientIntensity>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->AmbientIntensity;
		xml_stream << "</AmbientIntensity>" << std::endl;

		xml_stream << "<Shininess>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Shininess;
		xml_stream << "</Shininess>" << std::endl;

		xml_stream << "<DiffuseColor>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[0] << " ";
		xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[1] << " ";
		xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->DiffuseColor[2];
		xml_stream << "</DiffuseColor>" << std::endl;

		xml_stream << "<EmissiveColor>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[0] << " ";
		xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[1] << " ";
		xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->EmissiveColor[2];
		xml_stream << "</EmissiveColor>" << std::endl;

		xml_stream << "<SpecularColor>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[0] << " ";
		xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[1] << " ";
		xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->SpecularColor[2];
		xml_stream << "</SpecularColor>" << std::endl;

		if (((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Transparency == 0)
		{
			double transparency_tmp = 1;
			xml_stream << "<Transparency>" << transparency_tmp;
		}
		else
			xml_stream << "<Transparency>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Transparency;

		xml_stream << "</Transparency>" << std::endl;

		xml_stream << "</Material>" << std::endl;

		if (((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture)
		{
			//ÎÆÀí²ÎÊý
			xml_stream << "<GeoReferencedTexture>" << std::endl;

			xml_stream << "<ImageURI>" << (((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture)->ImageURI;
			xml_stream << "</ImageURI>" << std::endl;

			xml_stream << "<MimeType>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->MimeType;
			xml_stream << "</MimeType>" << std::endl;

			xml_stream << "<WrapMode>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->WrapMode;
			xml_stream << "</WrapMode>" << std::endl;

			xml_stream << "<Repeat>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->RepeatedCount;
			xml_stream << "</Repeat>" << std::endl;

			xml_stream << "<BorderColor>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->BorderColor[0] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->BorderColor[1] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->BorderColor[2] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->BorderColor[3];
			xml_stream << "</BorderColor>" << std::endl;
			xml_stream << "<ReferencePoint>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->ReferencePoint[0] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->ReferencePoint[1] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->ReferencePoint[2];
			xml_stream << "</ReferencePoint>" << std::endl;
			xml_stream << "<Orientation>" << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->Orientation[0] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->Orientation[1] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->Orientation[2] << " ";
			xml_stream << ((se::GeofaceSymbolizer*)(rule->GetSymbolizer()))->Texture->Orientation[3];
			xml_stream << "</Orientation>" << std::endl;
			xml_stream << "</GeoReferencedTexture>" << std::endl;
		}
		xml_stream << "</Front>" << std::endl;
		xml_stream << "<Back>" << std::endl;
		xml_stream << "</Back>" << std::endl;
	}
	xml_stream << "</" << xml_node_name << ">" << std::endl;
}
