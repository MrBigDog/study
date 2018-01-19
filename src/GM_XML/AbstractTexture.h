#ifndef AbstractTexture_h__
#define AbstractTexture_h__

#include "AbstractGML.h"
#include <string>

namespace gmml
{
	class GEOXML_EXPORT  AbstractTexture
	{
	public:
		AbstractTexture(void);
		~AbstractTexture(void);
		virtual std::string GetClassName() { return "AbstractTexture"; };

		std::string ImageURI;
		std::string MimeType;
		int         WrapMode;
		double      BorderColor[4];
		double ReferencePoint[3];
		double Orientation[4];
		// 
		int RepeatedCount;
		int FaceNorm;
	};

	class GEOXML_EXPORT GeoReferencedTexture : public AbstractTexture
	{
	public:
		GeoReferencedTexture(void) {};
		~GeoReferencedTexture(void) {};
		virtual std::string GetClassName() { return "GeoReferencedTexture"; };

	public:
		//double ReferencePoint[3] ;
		//double Orientation[4];
	};

	class GEOXML_EXPORT ParameterizedTexture : public AbstractTexture
	{
	public:
		ParameterizedTexture(void) {};
		~ParameterizedTexture(void) {};
		virtual std::string GetClassName() { return "ParameterizedTexture"; };

	public:
		double* TexCoordList;
		int     TexCoordListCount;
		double TexCoordGen[16];
	};
}

#endif // AbstractTexture_h__
