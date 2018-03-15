#include "GeneratorHelp.h"
#include <gwGeologicalUtil/MeshConsolidator.h>

#include <osg/TriangleIndexFunctor>
#include <sstream>

using namespace gwUtil;
//--------------------------------------------------------------------------------------
void ColumnGeometryOptimizer::apply(osg::Geode& geode)
{
	const osg::MatrixList& ml = geode.getWorldMatrices();
	osg::Matrixd localToWorld = ml.empty() ? osg::Matrixd::identity() : ml[0];
	osg::Matrixd inverseLocalToWorld = osg::Matrixd::inverse(localToWorld);

	osg::Plane plane = osg::Plane(_plane);
	plane.transform(inverseLocalToWorld);

	//计算纹理坐标、法线;   需要matrix转换;
	for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
	{
		osg::Geometry* ogeom = geode.getDrawable(i)->asGeometry();
		if (!ogeom) continue;

		osg::Vec3d wellCenter = _wellCenter*inverseLocalToWorld;
		osg::Vec3d sideDir = inverseLocalToWorld.getRotate()*_sideDir;
		osg::Vec3d dir = inverseLocalToWorld.getRotate()*_dir;

		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry(*ogeom, osg::CopyOp::DEEP_COPY_ALL);
		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		if (!va) continue;

		double dis1 = plane.distance(wellCenter);

		osg::ref_ptr<osg::Vec2Array> ta = new osg::Vec2Array;
		ta->reserve(va->size());
		osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array;
		na->reserve(va->size());
		for (osg::Vec3Array::const_iterator vait = va->begin(); vait != va->end(); ++vait)
		{
			double dis2 = plane.distance(*vait);
			osg::Vec3d nc = wellCenter + dir*(dis2 - dis1);

			osg::Vec3d normal = normalize(*vait - nc);
			na->push_back(-normal);

			double angle = std::abs(getAngle(sideDir, normal, dir));
			double strideNum = angle / _strideAngle;

			double tc_s = (strideNum*_d)*0.02;
			double tc_t = dis2*0.02;
			ta->push_back(osg::Vec2(tc_s, tc_t));
		}
		geom->setTexCoordArray(0, ta);
		geom->setNormalArray(na, osg::Array::BIND_PER_VERTEX);
		geode.replaceDrawable(ogeom, geom);
	}
}

//------------------------------------------------------------------------------------------------
void PlaneGeometryOptimizer::apply(osg::Geode & geode)
{
	const osg::MatrixList& ml = geode.getWorldMatrices();
	osg::Matrixd localToWorld = ml.empty() ? osg::Matrixd::identity() : ml[0];
	osg::Matrixd inverseLocalToWorld = osg::Matrixd::inverse(localToWorld);

	osg::Plane plane_s = osg::Plane(_plane_s);
	plane_s.transform(inverseLocalToWorld);
	osg::Plane plane_t = osg::Plane(_plane_t);
	plane_t.transform(inverseLocalToWorld);

	//计算纹理坐标、法线;
	for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
	{
		osg::Geometry* ogeom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
		if (!ogeom) continue;

		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry(*ogeom, osg::CopyOp::DEEP_COPY_ALL);
		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);

		osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		if (!va) continue;

		osg::ref_ptr<osg::Vec2Array> ta = new osg::Vec2Array;
		ta->reserve(va->size());
		osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array(1);
		(*na)[0] = _normal;
		//na->push_back(_normal);
		for (osg::Vec3Array::const_iterator vait = va->begin(); vait != va->end(); ++vait)
		{
			double dis_s = plane_s.distance(*vait)*0.02;
			double dis_t = plane_t.distance(*vait)*0.02;
			ta->push_back(osg::Vec2(dis_s, dis_t));
		}
		geom->setTexCoordArray(0, ta);
		geom->setNormalArray(na, osg::Array::BIND_OVERALL);
		geode.replaceDrawable(ogeom, geom);
	}
}

//-------------------------------------------------------------------------------------------------



