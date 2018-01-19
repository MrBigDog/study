#ifndef AbstractSpatialReferenceSystem_h__
#define AbstractSpatialReferenceSystem_h__

#include "AbstractGML.h"
#include <string>

namespace gmml
{
	class GEOXML_EXPORT AbstractSpatialReferenceSystem
	{
	public:
		AbstractSpatialReferenceSystem(void);
		~AbstractSpatialReferenceSystem(void);
		virtual std::string GetClassName() { return "AbstractSpatialReferenceSystem"; };
	};



	class GEOXML_EXPORT SpatialReferenceSystemUsingGeographicIdentifiers : public AbstractSpatialReferenceSystem
	{
	public:
		SpatialReferenceSystemUsingGeographicIdentifiers(void) {};
		~SpatialReferenceSystemUsingGeographicIdentifiers(void) {};
		virtual std::string GetClassName() { return "SpatialReferenceSystemUsingGeographicIdentifiers"; };

		std::string   Name;

	};

	class GEOXML_EXPORT CoordinateReferenceSystem : public AbstractSpatialReferenceSystem
	{
	public:
		CoordinateReferenceSystem(void) {};
		~CoordinateReferenceSystem(void) {};
		virtual std::string GetClassName() { return "CoordinateReferenceSystem"; };


		std::string CoordinateReferenceSystemIdentifier;
		std::string CoordinateSystemType;
		std::string CoordinateSystemIdentifier;

		std::string Parameter;
	};


	class GEOXML_EXPORT VerticalReferenceSystem : public AbstractSpatialReferenceSystem
	{
	public:
		VerticalReferenceSystem(void) {};
		~VerticalReferenceSystem(void) {};
		virtual std::string GetClassName() { return "VerticalReferenceSystem"; };

		std::string VerticalReferenceSystemIdentifier;
	};
}

#endif // AbstractSpatialReferenceSystem_h__
