#include "FeatureStyle.h"

using namespace se;
 
FeatureStyleRule::FeatureStyleRule()
{
		
		Symbolizer = 0;
}

FeatureStyleRule::~FeatureStyleRule()
{
	if(Symbolizer)
		delete Symbolizer;
	Symbolizer = 0;
}

void FeatureStyleRule::SetSymbolizer(GeoSymbolizer * s)
{
	Symbolizer =s;
}

GeoSymbolizer *FeatureStyleRule::GetSymbolizer()
{
	return Symbolizer;
}

FeatureType3DStyle::FeatureType3DStyle(void)
{

}
FeatureType3DStyle::~FeatureType3DStyle(void)
{
	deleteAllrules();
}

std::string FeatureType3DStyle::getFeatureTypeName() const
{
	return featureTypeName_;
}

void FeatureType3DStyle::setFeatureTypeName(const std::string& name)
{
	featureTypeName_ = name;
}
int   FeatureType3DStyle::getRuleCount()
{
	return Rules.size();
}
 
void FeatureType3DStyle::addRule(FeatureStyleRule* rule)
{
	Rules.push_back(rule);
}

FeatureStyleRule* FeatureType3DStyle::getRule(int i)
{
	return Rules[i];
}

void FeatureType3DStyle::deleteAllrules()
{
	for(int i=0; i < Rules.size(); i++)
	{

		delete Rules[i];
	}

	Rules.clear();
}
 
 
 
void FeatureType3DStyle::write(std::ostream& xmlStream) const
{
	std::string xmlNodeName = getXmlNodeName();
	xmlStream << "<" << xmlNodeName << ">" << std::endl;
	//writeExt(xmlStream);
	xmlStream << "</" << xmlNodeName << ">" << std::endl;
}

std::string FeatureType3DStyle::getXmlNodeName() const
{
	return std::string("se:FeatureType3DStyle");
}

void FeatureType3DStyle::writeExt(std::ostream& xmlStream) const
{
	 
}

 
 