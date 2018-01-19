#pragma once
#include "gmlfeature.h"
#include <vector>
class GMLFEATURE_EXPORT gmlFeatureCollection :public gmlFeature
{
public:
	gmlFeatureCollection(void);
	~gmlFeatureCollection(void);

	virtual char* GetClassName() = 0;
	virtual void  AddFeature(gmlFeature*);
	gmlFeature*   GetFeature(int i);
	long          GetFeatureCount();
	long          RemoveFeature(gmlFeature*);

protected:
	std::vector<gmlFeature*> mFeatures;
};

