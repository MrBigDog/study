#ifndef ASSISTPLANEGENERATOR_H
#define ASSISTPLANEGENERATOR_H 1

#include "Export.h"
#include <osg/Node>
#include <osg/Plane>

class GWGEOLOGICAL_EXPORT AssistPlaneGenerator
{
public:
	AssistPlaneGenerator(bool isOnEarth = false) :_isOnEarth(isOnEarth) {}

	osg::Node* operator()(const osg::BoundingBox& box, const osg::Plane& plane);
	osg::Node* operator()(const osg::BoundingBox& box, const std::vector<osg::Plane> planes);

private:
	bool _isOnEarth;
};

struct GWGEOLOGICAL_EXPORT FencePlaneGenerator
{
public:
	FencePlaneGenerator(bool isOnEarth = false) :_isOnEarth(isOnEarth) {}

	//πÃ∂®æ‡¿Î«–µ∂;
	osg::Node* operator()(const osg::BoundingBox& bb, const osg::Vec3d& slicedir, double deltaDis);
	//πÃ∂® ˝¡ø«–µ∂;
	osg::Node* operator()(const osg::BoundingBox& bb, const osg::Vec3d& slicedir, int sliceNum);

private:
	bool _isOnEarth;
};

#endif // ASSISTPLANEGENERATOR_H
