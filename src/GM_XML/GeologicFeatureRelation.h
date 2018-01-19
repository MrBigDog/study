#pragma once

#include "GeologicRelation.h"

using namespace gmml;

namespace gsml
{
class gsGeoFeature;
class GeologicFeatureRelation : public GeologicRelation
{
public:
	GeologicFeatureRelation(void);
	~GeologicFeatureRelation(void);

	//@{
	void setRelatedFeature(const std::string& id);
	std::string getRelatedFeature() const;
	//@}
	gsGeoFeature* getRelatedFeaturePointer();
    void setRelatedFeaturePointer(gsGeoFeature* f    );
protected:
	virtual std::string getXmlNodeName() const;
	virtual void writeExt(std::ostream& xmlStream) const;

private:
	std::string relatedFeatureID_;
	gsGeoFeature* relatedFeature_;
};

}