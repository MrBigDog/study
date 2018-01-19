#include "CoverageStyle.h"

using namespace se;
 
CoverageStyleRule::CoverageStyleRule()
{
		
		Symbolizer = 0;
}

CoverageStyleRule::~CoverageStyleRule()
{
	if(Symbolizer)
		delete Symbolizer;
	Symbolizer = 0;
}

void CoverageStyleRule::SetSymbolizer(GeoSymbolizer * s)
{
	Symbolizer =s;
}

GeoSymbolizer *CoverageStyleRule::GetSymbolizer()
{
	return Symbolizer;
}

CoverageType3DStyle::CoverageType3DStyle(void)
{

}
CoverageType3DStyle::~CoverageType3DStyle(void)
{
	deleteAllrules();
}

std::string CoverageType3DStyle::getFeatureTypeName() const
{
	return featureTypeName_;
}

void CoverageType3DStyle::setFeatureTypeName(const std::string& name)
{
	featureTypeName_ = name;
}
int   CoverageType3DStyle::getRuleCount()
{
	return Rules.size();
}
 
void CoverageType3DStyle::addRule(CoverageStyleRule* rule)
{
	Rules.push_back(rule);
}

CoverageStyleRule* CoverageType3DStyle::getRule(int i)
{
	return Rules[i];
}

void CoverageType3DStyle::deleteAllrules()
{
	for(int i=0; i < Rules.size(); i++)
	{

		delete Rules[i];
	}

	Rules.clear();
}
 
 
 
void CoverageType3DStyle::write(std::ostream& xmlStream) const
{
	std::string xmlNodeName = getXmlNodeName();
	xmlStream << "<" << xmlNodeName << ">" << std::endl;
	//writeExt(xmlStream);
	xmlStream << "</" << xmlNodeName << ">" << std::endl;
}

std::string CoverageType3DStyle::getXmlNodeName() const
{
	return std::string("se:CoverageTypeStyle");
}

void CoverageType3DStyle::writeExt(std::ostream& xmlStream) const
{
	 
}

 
 