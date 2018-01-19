#include "GeologicFeatureRelation.h"

using namespace gsml;

GeologicFeatureRelation::GeologicFeatureRelation(void)
{
}

GeologicFeatureRelation::~GeologicFeatureRelation(void)
{
}
	gsGeoFeature *GeologicFeatureRelation::getRelatedFeaturePointer()  
	{
		return relatedFeature_;
	}
void GeologicFeatureRelation::setRelatedFeaturePointer(gsGeoFeature* f    )
{
	relatedFeature_ = f;
}
void GeologicFeatureRelation::setRelatedFeature(const std::string& id)
{
	relatedFeatureID_ = id;
}

std::string GeologicFeatureRelation::getRelatedFeature() const
{
	return relatedFeatureID_;
}

std::string GeologicFeatureRelation::getXmlNodeName() const
{
	return std::string("gsml:GeologicFeatureRelation");
}

void GeologicFeatureRelation::writeExt(std::ostream& xmlStream) const
{
	GeologicRelation::writeExt(xmlStream);
	xmlStream << "<gsml:relatedFeature xlink:type=\"simple\" xlink:href=\"#" << getRelatedFeature() << "\" />" << std::endl;
}