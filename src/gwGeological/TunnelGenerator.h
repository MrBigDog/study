#ifndef GENERATETUNELOPERATOR_H
#define GENERATETUNELOPERATOR_H 1

#include "Export.h"
#include "SlicePlane.h"
#include <osg/Array>
#include <osg/Group>

//创建方形隧道（拱形顶/平顶）;
class GWGEOLOGICAL_EXPORT SquareTunelGenerator
{
public:
	enum TopType { ARC_TOP, PLANE_TOP };

	SquareTunelGenerator(bool isOnEarth = false) :_isOnEarth(isOnEarth) {}
	osg::Node* operator()(osg::Node* geologicalGroup, const osg::Vec3dArray* va, double width, double height, TopType topType = ARC_TOP);

	//生成裁剪面;
	static void generateSlicePlanes(const osg::Vec3dArray* va, double width, double height, TopType topType, bool isOnEarth, SlicePLaneVector& out_slice_planes);

private:
	bool _isOnEarth;
};

//创建圆形隧道;
class GWGEOLOGICAL_EXPORT CircleTunelGenerator
{
public:
	CircleTunelGenerator(bool isOnEarth = false) :_isOnEarth(isOnEarth) {}
	osg::Node* operator()(osg::Node* geologicalGroup, const osg::Vec3dArray* va, double radius);

	//生成裁剪面;
	static void generateSlicePlanes(const osg::Vec3dArray* va, double radius, bool isOnEarth, SlicePLaneVector& out_slice_planes);

private:
	bool _isOnEarth;
};

//创建壕沟;
class GWGEOLOGICAL_EXPORT TrenchGenerator
{
public:
	TrenchGenerator(bool isOnEarth = false) :_isOnEarth(isOnEarth) {}

	osg::Node* operator()(osg::Node*geoGroup, const osg::Vec3dArray* va, double vsize, double grade);

	//生成裁剪面;
	static void generateSlicePlanes(const osg::Vec3dArray* va, double vsize, bool isOnEarth, SlicePLaneVector& out_slice_planes);

private:
	bool _isOnEarth;
};


#endif // GenerateTunelOperator_h__
