#ifndef VOXELUTIL_H
#define VOXELUTIL_H 1

#include <osg/Node>
#include <osg/Array>

void getVoxelInTunel(osg::Vec3dArray* va, float r, osg::Node* geoModel, std::vector<osg::ref_ptr<osg::Node> >& outResult);


#endif // VoxelUtil_h__
