#ifndef GeoSymbolizer_h__
#define GeoSymbolizer_h__

#include "export.h"
#include <vector>
#include <string>

namespace gmml
{
	class AbstractTexture;
}

namespace se
{
	class GEOXML_EXPORT GeoSymbolizer
	{
	public:
		GeoSymbolizer(void);
		~GeoSymbolizer(void);
		virtual std::string GetClassName() { return "GeoSymbolizer"; };


	public:
		double AmbientIntensity;
		double Shininess;
		double Transparency;

		double DiffuseColor[3];
		double EmissiveColor[3];
		double SpecularColor[3];
		bool Smooth;
	};

	class GEOXML_EXPORT GeoPointSymbolizer : public GeoSymbolizer
	{
	public:

		virtual std::string GetClassName() { return "GeoPointSymbolizer"; };

	public:
		int PointSize;
	};

	class GeoLineSymbolizer : public GeoSymbolizer
	{
	public:

		virtual std::string GetClassName() { return "GeoLineSymbolizer"; };

	public:
		int LineWidth;
	};



	class GEOXML_EXPORT GeofaceSymbolizer : public GeoSymbolizer
	{
	public:
		GeofaceSymbolizer(void);
		~GeofaceSymbolizer(void);
		virtual std::string GetClassName() { return "GeofaceSymbolizer"; };
		void SetTexture(gmml::AbstractTexture* t);
	public:
		GeoLineSymbolizer*  EdgeSymbolizer;
		GeoPointSymbolizer* PointSymbolizer;
		GeofaceSymbolizer*  Back;

		gmml::AbstractTexture*    Texture;
	};


	class GEOXML_EXPORT RasterSymbolizer : public GeoSymbolizer
	{
	public:
		struct ColorMapEntry
		{
			double Quanlity;
			std::string   ColorRef;//
			double        Opacity;
			std::string   Label;

		};
		RasterSymbolizer(void);
		~RasterSymbolizer(void);
		virtual std::string GetClassName() { return "RasterSymbolizer"; };
		int GetType();
		void SetType(int type);
		void CreateMap(int ColormapEntryCount);
		void SetGeometry(std::string    geometry);
		std::string    GetGeometry();

		void AddMapEntry(double Quanlity, std::string   ColorRef, double        Opacity, std::string   Label);
		void GetMapEntry(int index, double &Quanlity, std::string  & ColorRef, double &       Opacity, std::string  & Label);
		int  GetMapEntryCount();
	public:
		int MapType; // 0 Ramp 1 interval 2 values
		std::vector<ColorMapEntry> ColormapEntries;
		//int            ColormapEntryCount;
		std::string    Geometry; //propertyName
	};

	class CoverageStyleRule;
	class GEOXML_EXPORT GeoDiscreteCoverageSymbolizer : public GeoSymbolizer
	{
	public:
		GeoDiscreteCoverageSymbolizer(void) {};
		~GeoDiscreteCoverageSymbolizer();
		virtual std::string GetClassName() { return "GeoDiscreteCoverageSymbolizer"; };

		int getRuleCount();

		void addRule(CoverageStyleRule* rule);
		CoverageStyleRule* getRule(int i);
		void deleteAllrules();
		RasterSymbolizer* GetRasterSymbolizer(std::string propname);
	public:
		std::vector<CoverageStyleRule*> Rules;
	};
}
#endif // GeoSymbolizer_h__