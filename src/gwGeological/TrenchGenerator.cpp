#include "TrenchGenerator.h"
#include "TunnelGenerator.h"

namespace
{
	void generateSplitPlanes(osg::Vec3Array* va, double vsize)
	{

	}

	osg::Node* generateTrenchSegment(osg::Node* geoGroup, const osg::Plane& splane, const osg::Plane& eplane)
	{

	}

}

TrenchGenerator::~TrenchGenerator()
{}

osg::Node * TrenchGenerator::operator()(osg::Node * geologicalGroup, const osg::Vec3dArray * va, double width, double height)
{
	if (!va || !geologicalGroup) return 0L;
	if (va->size() < 2) return 0L;



	return nullptr;
}

osg::Node * TrenchGenerator::operator()(osg::Node * geoGroup, const osg::Vec3dArray * va, double vsize)
{
	return nullptr;
}

void TrenchGenerator::generateSlicePlanes(const osg::Vec3dArray * va, double radius, bool isOnEarth, SlicePLaneVector & out_slice_planes)
{

}
