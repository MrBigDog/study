#include "GeoSymbolizer.h"
#include "CoverageStyle.h"
using namespace se;
GeoSymbolizer::GeoSymbolizer(void)
{
	Smooth = 0;

	AmbientIntensity = 0;
	Shininess = 0;
	Transparency = 0;

	DiffuseColor[0] =DiffuseColor[1] =DiffuseColor[2] = 0;
	EmissiveColor[0] = EmissiveColor[1] =EmissiveColor[2] = 0;
	SpecularColor[0] =SpecularColor[1] =SpecularColor[2] = 0;
  
}


GeoSymbolizer::~GeoSymbolizer(void)
{
	
}

GeofaceSymbolizer::GeofaceSymbolizer(void)
{
	Smooth = 0;

	EdgeSymbolizer = 0;
	PointSymbolizer = 0;
	Back = 0;

	 Texture = 0;

}



void GeofaceSymbolizer::SetTexture( gmml::AbstractTexture* t)
{
	if(Texture)
		delete Texture;

	 Texture = t;
}
GeofaceSymbolizer::~GeofaceSymbolizer(void)
{
    if(EdgeSymbolizer)
		delete EdgeSymbolizer;

	if(PointSymbolizer)
		delete PointSymbolizer;

	if(Back)
		delete Back;

	if(Texture)
		delete Texture;

	EdgeSymbolizer = 0;
	PointSymbolizer = 0;
	Back = 0;
}
RasterSymbolizer::RasterSymbolizer()
{
MapType = 0;
//ColormapEntries = 0;
//ColormapEntryCount = 0;
}

RasterSymbolizer::~RasterSymbolizer()
{
	//if(ColormapEntries)
	//	delete[] ColormapEntries;
//	ColormapEntries = 0;
}

int RasterSymbolizer::GetType()
{
	
	return MapType;
}
void RasterSymbolizer::SetType(int t)
{
	MapType = t;
}

std::string   RasterSymbolizer::GetGeometry()
{

	return Geometry;
}
void RasterSymbolizer::SetGeometry(std::string   t)
{
	Geometry = t;
}

void RasterSymbolizer::CreateMap(int c)
{
//	if(ColormapEntries)
//		delete[] ColormapEntries;
//	ColormapEntries = 0;
//	ColormapEntryCount = c;
//	ColormapEntries = new ColorMapEntry[c];
}

int  RasterSymbolizer::GetMapEntryCount()
{
	return ColormapEntries.size();
}
void RasterSymbolizer:: AddMapEntry(double Quanlity, std::string   ColorRef,double        Opacity,std::string   Label)
{
	 ColorMapEntry ColormapEntr;

	ColormapEntr.ColorRef = ColorRef;
	ColormapEntr.Quanlity = Quanlity;
	ColormapEntr.Opacity = Opacity;
	ColormapEntr.Label = Label;

	ColormapEntries.push_back(ColormapEntr);

}
void RasterSymbolizer::GetMapEntry(int index,double &Quanlity, std::string  & ColorRef,double &       Opacity,std::string  & Label)
{
		if(index<0)return;
	if(index>ColormapEntries.size()-1)return;

	ColorRef=	ColormapEntries[index].ColorRef ;
	Quanlity = ColormapEntries[index].Quanlity  ;
	Opacity = ColormapEntries[index].Opacity ;
	Label = ColormapEntries[index].Label  ;
}

GeoDiscreteCoverageSymbolizer::~GeoDiscreteCoverageSymbolizer()
{

	deleteAllrules();
}
void GeoDiscreteCoverageSymbolizer::addRule(CoverageStyleRule* rule)
{
	Rules.push_back(rule);
}

CoverageStyleRule* GeoDiscreteCoverageSymbolizer::getRule(int i)
{
	return Rules[i];
}

int GeoDiscreteCoverageSymbolizer::getRuleCount()
{
	return Rules.size();
}
void GeoDiscreteCoverageSymbolizer::deleteAllrules()
{
	for(int i=0; i < Rules.size(); i++)
	{

		delete Rules[i];
	}

	Rules.clear();
}

 RasterSymbolizer*   GeoDiscreteCoverageSymbolizer::GetRasterSymbolizer(std::string propname)
 {
	 for(int i=0; i < Rules.size(); i++)
	 {

		  if( Rules[i]->GetSymbolizer() && Rules[i]->GetSymbolizer()->GetClassName()=="RasterSymbolizer")
		  {
			 RasterSymbolizer* pp = (RasterSymbolizer* )Rules[i]->GetSymbolizer();

			 if(pp->GetGeometry() == propname)
				 return pp;
		  }


	 }

	 return 0;
 }
 
 
  

 
 