/************************************************************************/
/* 程序实现：BigDog                                                      */
/* 参考文章：《参数曲线集复杂区域的全自动识别算法》-谭正华                   ;*/
/************************************************************************/
#ifndef GWGEOLOGICAL_SLICEOPERATOR_H
#define GWGEOLOGICAL_SLICEOPERATOR_H 1

#include "Export.h"
#include <gwGeologicalUtil/Random.h>

#include <osg/Vec3d>
#include <osg/Array>
#include <osg/Plane>
#include <osg/Group>
#include <osg/Texture2D>
#include <osg/BoundingBox>

#include <vector>
#include <algorithm>
//
typedef std::vector<osg::ref_ptr<osg::Vec3dArray> > Vec3dArrayVector;

//////////////////////////////////////////////////////////////////////////
class GWGEOLOGICAL_EXPORT SliceOperator
{
	enum ModelType
	{
		VOXEL, HULL
	};

public:
	SliceOperator(bool isOnEarth = false, ModelType modelType = VOXEL);
	~SliceOperator();

	osg::Node* slice(osg::Node* geoModel, const osg::Plane& slicePlane);
	osg::Node* sliceGeom(osg::Geometry* geom, const osg::Plane& sliceplane);

private:
	osg::Node* sliceNode(osg::Node* geologicalModel, const osg::Plane& slicePlane);
	osg::Node* sliceGeometry(osg::Geometry* geom, const osg::Plane& sliceplane);

private:
	bool _isOnEarth;
	ModelType _modelType;
	gwUtil::Random _random;
};

#endif // SliceOperator_h__
