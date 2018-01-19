#ifndef SLICEPLANE_H
#define SLICEPLANE_H 1

#include "Export.h"
#include <osg/Plane>
#include <osg/Polytope>
#include <vector>

typedef std::vector<osg::Plane> PlaneVec;

struct SlicePlane
{
	SlicePlane(const osg::Plane& plane, const osg::Polytope& polytope)
		:_plane(plane), _polytope(polytope)
	{}

	osg::Plane _plane;
	osg::Polytope _polytope;
};
typedef std::vector<SlicePlane> SlicePLaneVector;
typedef std::vector<SlicePlane>::const_iterator SlicePlaneVectorConstIt;

//////////////////////////////////////////////////////////////////////////
class GWGEOLOGICAL_EXPORT SlicePlaneNodeGenerator :public osg::Referenced
{
public:
	SlicePlaneNodeGenerator() {}
	osg::Node* operator()(osg::Node* singleHullNode, bool isOnEarth, const SlicePLaneVector& slicePlanes);
};

#endif // SlicePlane_h__