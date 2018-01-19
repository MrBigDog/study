#pragma once
#include "xmlreader.h"
class GMXMLFILEIO_EXPORT FeatureXMLReader :
	public XMLReader
{
public:
	FeatureXMLReader(void);
	~FeatureXMLReader(void);
	gml::AbstractGML* GetFeatureRelation();
 
	virtual bool ReadFeatureRelation(xmlNodePtr curNode){return true;};//add by zyx

protected:
	void CreateFeatureRelation();
	gml::AbstractGML* FeatureRelation_;
};

