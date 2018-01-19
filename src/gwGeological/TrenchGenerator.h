#ifndef GWGEOLOGICAL_TRENCHGENERATOR_H
#define GWGEOLOGICAL_TRENCHGENERATOR_H 1

#include <gwGeological/Export.h>
#include <gwGeological/SlicePlane.h>
#include <osg/Node>

class GWGEOLOGICAL_EXPORT TrenchGenerator
{
public:
	TrenchGenerator(bool isOnEarth = false) :_isOnEarth(isOnEarth) {}
	~TrenchGenerator();

	osg::Node* operator()(osg::Node* geologicalGroup, const osg::Vec3dArray* va, double width, double height);

	static void generateSlicePlanes(const osg::Vec3dArray* va, double radius, bool isOnEarth, SlicePLaneVector& out_slice_planes);


private:
	bool _isOnEarth;
};


#endif // TrenchGenerator_h__
