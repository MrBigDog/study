#ifndef GeologicRelation_h__
#define GeologicRelation_h__

#include "FeatureRelation.h"

namespace gmml
{
	class GEOXML_EXPORT GeologicRelation : public FeatureRelation
	{
	public:
		GeologicRelation(void);
		~GeologicRelation(void);
		virtual std::string GetClassName() { return "GeologicRelation"; };

	};

	class GEOXML_EXPORT GeologicHistory : public FeatureRelation
	{
	public:
		GeologicHistory(void) {};
		~GeologicHistory(void) {};
		virtual std::string GetClassName() { return "GeologicHistory"; };

	};


	class GEOXML_EXPORT AggregationRelation : public FeatureRelation
	{
	public:
		AggregationRelation(void) {};
		~AggregationRelation(void) {};
		virtual std::string GetClassName() { return "AggregationRelation"; };

	};
	class GEOXML_EXPORT BoundaryRelation : public FeatureRelation
	{
	public:
		BoundaryRelation(void) {};
		~BoundaryRelation(void) {};
		virtual std::string GetClassName() { return "BoundaryRelationship"; };

	};

	class GEOXML_EXPORT DefiningStructure : public FeatureRelation
	{
	public:
		DefiningStructure(void) {};
		~DefiningStructure(void) {};
		virtual std::string GetClassName() { return "DefiningStructure"; };

	};
}

#endif // GeologicRelation_h__