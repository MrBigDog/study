/************************************************************************/
///闭合模型切割并补剖面(BigDog 2017.04.19);
/************************************************************************/
#ifndef GEOLOGICALTOOL_CUTTER_H
#define GEOLOGICALTOOL_CUTTER_H 1

#include <gwGeological/SliceOperator.h>
#include <gwGeologicalUtil/MeshConsolidator.h>

#include <osg/Polytope>
#include <osg/MatrixTransform>
#include <osg/TriangleIndexFunctor>
#include <osgDB/WriteFile>
#include <osgUtil/MeshOptimizers>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/PlaneIntersector>
#include <iostream>

struct TriangulateOperator
{
public:
	enum Side { POSITIVE_SIDE, NEGATIVE_SIDE };

public:
	TriangulateOperator()
		: vAry_(0L), nAry_(0L), cAry_(0L), tAry_(0L)
		, rcAry_(0L), rvAry_(0L), rnAry_(0L), rtAry_(0L)
		, lcAry_(0L), lvAry_(0L), lnAry_(0L), ltAry_(0L)
	{ }

	void getGeomProperties(const osg::Geometry& geom)
	{
		geom_ = new osg::Geometry(geom);

		const osg::Vec3Array* va = dynamic_cast<const osg::Vec3Array*>(geom.getVertexArray());
		if (!va) return;
		vAry_ = va;
		rvAry_ = new osg::Vec3Array;
		lvAry_ = new osg::Vec3Array;

		const osg::Vec3Array* na = dynamic_cast<const osg::Vec3Array*>(geom.getNormalArray());
		if (na)
		{
			nAry_ = na;
			rnAry_ = new osg::Vec3Array;
			lnAry_ = new osg::Vec3Array;
		}
		const osg::Vec2Array* ta = dynamic_cast<const osg::Vec2Array*>(geom.getTexCoordArray(0));
		if (ta)
		{
			tAry_ = ta;
			rtAry_ = new osg::Vec2Array;
			ltAry_ = new osg::Vec2Array;
		}
		const osg::Vec4Array* ca = dynamic_cast<const osg::Vec4Array*>(geom.getColorArray());
		if (ca)
		{
			cAry_ = ca;
			rcAry_ = new osg::Vec4Array;
			lcAry_ = new osg::Vec4Array;
		}
	}

	void setCutPlane(const osg::Plane& plane)
	{
		plane_ = plane;
	}

	osg::Geometry* getGeometry(Side side)
	{
		osg::ref_ptr<osg::Vec3Array> va = side == POSITIVE_SIDE ? lvAry_ : rvAry_;
		osg::ref_ptr<osg::Vec3Array> na = side == POSITIVE_SIDE ? lnAry_ : rnAry_;
		osg::ref_ptr<osg::Vec2Array> ta = side == POSITIVE_SIDE ? ltAry_ : rtAry_;
		osg::ref_ptr<osg::Vec4Array> ca = side == POSITIVE_SIDE ? lcAry_ : rcAry_;

		if (!va) return 0L;
		if (va->size() < 3) return 0L;

		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		geom->setVertexArray(va);

		int vs = va->size();
		if (ta && ta->size() == vs)geom->setTexCoordArray(0, ta);
		if (na && na->size() == vs)geom->setNormalArray(na, osg::Array::BIND_PER_VERTEX);
		if (ca && ca->size() == vs)geom->setColorArray(ca, osg::Array::BIND_PER_VERTEX);
		else//防止合并Geometry的时候出问题;
		{
			ca = new osg::Vec4Array;
			ca->push_back(osg::Vec4(1, 1, 1, 1));
			geom->setColorArray(ca, osg::Array::BIND_OVERALL);
		}

		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vs));
		geom->setStateSet(geom_->getStateSet());
		geom->setUserData(geom_->getUserData());

		return geom.release();
	}

	//添加顶点的顺序不能变，否则会影响背面裁剪;
	inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3)
	{
		const osg::Vec3d& v1 = (*vAry_)[p1];
		const osg::Vec3d& v2 = (*vAry_)[p2];
		const osg::Vec3d& v3 = (*vAry_)[p3];
		double dis1 = plane_.distance(v1);
		double dis2 = plane_.distance(v2);
		double dis3 = plane_.distance(v3);

		int pointNumOnLeftSide = ((dis1 > 0.0) ? 1 : 0) + ((dis2 > 0.0) ? 1 : 0) + ((dis3 > 0.0) ? 1 : 0);
		if (pointNumOnLeftSide == 3)
		{
			addToVertexArray(v1, v2, v3, POSITIVE_SIDE);
			if (nAry_) addToNormalArray(nAry_->at(p1), nAry_->at(p2), nAry_->at(p3), POSITIVE_SIDE);
			if (cAry_) addToColorArray(getColor(cAry_, p1), getColor(cAry_, p2), getColor(cAry_, p3), POSITIVE_SIDE);
			if (tAry_) addToTextCoordArray(tAry_->at(p1), tAry_->at(p2), tAry_->at(p3), POSITIVE_SIDE);
		}
		else if (pointNumOnLeftSide == 0)
		{
			addToVertexArray(v1, v2, v3, NEGATIVE_SIDE);
			if (nAry_) addToNormalArray(nAry_->at(p1), nAry_->at(p2), nAry_->at(p3), NEGATIVE_SIDE);
			if (cAry_) addToColorArray(getColor(cAry_, p1), getColor(cAry_, p2), getColor(cAry_, p3), NEGATIVE_SIDE);
			if (tAry_) addToTextCoordArray(tAry_->at(p1), tAry_->at(p2), tAry_->at(p3), NEGATIVE_SIDE);
		}
		else if (pointNumOnLeftSide == 1)
		{
			if (dis1 > 0.0)
			{
				double r12 = abs(dis1) / (abs(dis1) + abs(dis2));
				double r13 = abs(dis1) / (abs(dis1) + abs(dis3));
				osg::Vec3d v12 = getLerp(v1, v2, r12);
				osg::Vec3d v13 = getLerp(v1, v3, r13);
				addToVertexArray(v1, v12, v13, POSITIVE_SIDE);
				addToVertexArray(v12, v2, v3, NEGATIVE_SIDE);
				addToVertexArray(v3, v13, v12, NEGATIVE_SIDE);
				if (nAry_)
				{
					osg::Vec3d n12 = getLerp(nAry_->at(p1), nAry_->at(p2), r12);
					osg::Vec3d n13 = getLerp(nAry_->at(p1), nAry_->at(p3), r13);
					addToNormalArray(nAry_->at(p1), n12, n13, POSITIVE_SIDE);
					addToNormalArray(n12, nAry_->at(p2), nAry_->at(p3), NEGATIVE_SIDE);
					addToNormalArray(nAry_->at(p3), n13, n12, NEGATIVE_SIDE);
				}
				if (cAry_)
				{
					osg::Vec4 c12 = getLerp(getColor(cAry_, p1), getColor(cAry_, p2), r12);
					osg::Vec4 c13 = getLerp(getColor(cAry_, p1), getColor(cAry_, p3), r13);
					addToColorArray(getColor(cAry_, p1), c12, c13, POSITIVE_SIDE);
					addToColorArray(c12, getColor(cAry_, p2), getColor(cAry_, p3), NEGATIVE_SIDE);
					addToColorArray(getColor(cAry_, p3), c13, c12, NEGATIVE_SIDE);
				}
				if (tAry_)
				{
					osg::Vec2 t12 = getLerp(tAry_->at(p1), tAry_->at(p2), r12);
					osg::Vec2 t13 = getLerp(tAry_->at(p1), tAry_->at(p3), r13);
					addToTextCoordArray(tAry_->at(p1), t12, t13, POSITIVE_SIDE);
					addToTextCoordArray(t12, tAry_->at(p2), tAry_->at(p3), NEGATIVE_SIDE);
					addToTextCoordArray(tAry_->at(p3), t13, t12, NEGATIVE_SIDE);
				}
			}
			else if (dis2 > 0.0)
			{
				double r12 = abs(dis1) / (abs(dis1) + abs(dis2));
				double r23 = abs(dis2) / (abs(dis2) + abs(dis3));
				osg::Vec3d v12 = getLerp(v1, v2, r12);
				osg::Vec3d v23 = getLerp(v2, v3, r23);
				addToVertexArray(v12, v2, v23, POSITIVE_SIDE);
				addToVertexArray(v1, v12, v23, NEGATIVE_SIDE);
				addToVertexArray(v1, v23, v3, NEGATIVE_SIDE);
				if (nAry_)
				{
					osg::Vec3d n12 = getLerp(nAry_->at(p1), nAry_->at(p2), r12);
					osg::Vec3d n23 = getLerp(nAry_->at(p2), nAry_->at(p3), r23);
					addToNormalArray(n12, nAry_->at(p2), n23, POSITIVE_SIDE);
					addToNormalArray(nAry_->at(p1), n12, n23, NEGATIVE_SIDE);
					addToNormalArray(nAry_->at(p1), n23, nAry_->at(p3), NEGATIVE_SIDE);
				}
				if (cAry_)
				{
					osg::Vec4 c12 = getLerp(getColor(cAry_, p1), getColor(cAry_, p2), r12);
					osg::Vec4 c23 = getLerp(getColor(cAry_, p2), getColor(cAry_, p3), r23);
					addToColorArray(c12, getColor(cAry_, p2), c23, POSITIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), c12, c23, NEGATIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), c23, getColor(cAry_, p3), NEGATIVE_SIDE);
				}
				if (tAry_)
				{
					osg::Vec2 t12 = getLerp(tAry_->at(p1), tAry_->at(p2), r12);
					osg::Vec2 t23 = getLerp(tAry_->at(p2), tAry_->at(p3), r23);
					addToTextCoordArray(t12, tAry_->at(p2), t23, POSITIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), t12, t23, NEGATIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), t23, tAry_->at(p3), NEGATIVE_SIDE);
				}
			}
			else if (dis3 > 0.0)
			{
				double r13 = abs(dis1) / (abs(dis1) + abs(dis3));
				double r23 = abs(dis2) / (abs(dis2) + abs(dis3));
				osg::Vec3d v13 = getLerp(v1, v3, r13);
				osg::Vec3d v23 = getLerp(v2, v3, r23);
				addToVertexArray(v3, v13, v23, POSITIVE_SIDE);
				addToVertexArray(v1, v2, v23, NEGATIVE_SIDE);
				addToVertexArray(v1, v23, v13, NEGATIVE_SIDE);
				if (nAry_)
				{
					osg::Vec3d n13 = getLerp(nAry_->at(p1), nAry_->at(p3), r13);
					osg::Vec3d n23 = getLerp(nAry_->at(p2), nAry_->at(p3), r23);
					addToNormalArray(nAry_->at(p3), n13, n23, POSITIVE_SIDE);
					addToNormalArray(nAry_->at(p1), nAry_->at(p2), n23, NEGATIVE_SIDE);
					addToNormalArray(nAry_->at(p1), n23, n13, NEGATIVE_SIDE);
				}
				if (cAry_)
				{
					osg::Vec4 c13 = getLerp(getColor(cAry_, p1), getColor(cAry_, p3), r13);
					osg::Vec4 c23 = getLerp(getColor(cAry_, p2), getColor(cAry_, p3), r23);
					addToColorArray(getColor(cAry_, p3), c13, c23, POSITIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), getColor(cAry_, p2), c23, NEGATIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), c23, c13, NEGATIVE_SIDE);
				}
				if (tAry_)
				{
					osg::Vec2 t13 = getLerp(tAry_->at(p1), tAry_->at(p3), r13);
					osg::Vec2 t23 = getLerp(tAry_->at(p2), tAry_->at(p3), r23);
					addToTextCoordArray(tAry_->at(p3), t13, t23, POSITIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), tAry_->at(p2), t23, NEGATIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), t23, t13, NEGATIVE_SIDE);
				}
			}
		}
		else if (pointNumOnLeftSide == 2)
		{
			if (dis1 < 0.0)
			{
				double r12 = abs(dis1) / (abs(dis1) + abs(dis2));
				double r13 = abs(dis1) / (abs(dis1) + abs(dis3));
				osg::Vec3d v12 = getLerp(v1, v2, r12);
				osg::Vec3d v13 = getLerp(v1, v3, r13);
				addToVertexArray(v1, v12, v13, NEGATIVE_SIDE);
				addToVertexArray(v12, v2, v3, POSITIVE_SIDE);
				addToVertexArray(v3, v13, v12, POSITIVE_SIDE);
				if (nAry_)
				{
					osg::Vec3d n12 = getLerp(nAry_->at(p1), nAry_->at(p2), r12);
					osg::Vec3d n13 = getLerp(nAry_->at(p1), nAry_->at(p3), r13);
					addToNormalArray(nAry_->at(p1), n12, n13, NEGATIVE_SIDE);
					addToNormalArray(n12, nAry_->at(p2), nAry_->at(p3), POSITIVE_SIDE);
					addToNormalArray(nAry_->at(p3), n13, n12, POSITIVE_SIDE);
				}
				if (cAry_)
				{
					osg::Vec4 c12 = getLerp(getColor(cAry_, p1), getColor(cAry_, p2), r12);
					osg::Vec4 c13 = getLerp(getColor(cAry_, p1), getColor(cAry_, p3), r13);
					addToColorArray(getColor(cAry_, p1), c12, c13, NEGATIVE_SIDE);
					addToColorArray(c12, getColor(cAry_, p2), getColor(cAry_, p3), POSITIVE_SIDE);
					addToColorArray(getColor(cAry_, p3), c13, c12, POSITIVE_SIDE);
				}
				if (tAry_)
				{
					osg::Vec2 t12 = getLerp(tAry_->at(p1), tAry_->at(p2), r12);
					osg::Vec2 t13 = getLerp(tAry_->at(p1), tAry_->at(p3), r13);
					addToTextCoordArray(tAry_->at(p1), t12, t13, NEGATIVE_SIDE);
					addToTextCoordArray(t12, tAry_->at(p2), tAry_->at(p3), POSITIVE_SIDE);
					addToTextCoordArray(tAry_->at(p3), t13, t12, POSITIVE_SIDE);
				}
			}
			else if (dis2 < 0.0)
			{
				double r12 = abs(dis1) / (abs(dis1) + abs(dis2));
				double r23 = abs(dis2) / (abs(dis2) + abs(dis3));
				osg::Vec3d v12 = getLerp(v1, v2, r12);
				osg::Vec3d v23 = getLerp(v2, v3, r23);
				addToVertexArray(v12, v2, v23, NEGATIVE_SIDE);
				addToVertexArray(v1, v12, v23, POSITIVE_SIDE);
				addToVertexArray(v1, v23, v3, POSITIVE_SIDE);
				if (nAry_)
				{
					osg::Vec3d n12 = getLerp(nAry_->at(p1), nAry_->at(p2), r12);
					osg::Vec3d n23 = getLerp(nAry_->at(p2), nAry_->at(p3), r23);
					addToNormalArray(n12, nAry_->at(p2), n23, NEGATIVE_SIDE);
					addToNormalArray(nAry_->at(p1), n12, n23, POSITIVE_SIDE);
					addToNormalArray(nAry_->at(p1), n23, nAry_->at(p3), POSITIVE_SIDE);
				}
				if (cAry_)
				{
					osg::Vec4 c12 = getLerp(getColor(cAry_, p1), getColor(cAry_, p2), r12);
					osg::Vec4 c23 = getLerp(getColor(cAry_, p2), getColor(cAry_, p3), r23);
					addToColorArray(c12, getColor(cAry_, p2), c23, NEGATIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), c12, c23, POSITIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), c23, getColor(cAry_, p3), POSITIVE_SIDE);
				}
				if (tAry_)
				{
					osg::Vec2 t12 = getLerp(tAry_->at(p1), tAry_->at(p2), r12);
					osg::Vec2 t23 = getLerp(tAry_->at(p2), tAry_->at(p3), r23);
					addToTextCoordArray(t12, tAry_->at(p2), t23, NEGATIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), t12, t23, POSITIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), t23, tAry_->at(p3), POSITIVE_SIDE);
				}
			}
			else if (dis3 < 0.0)
			{
				double r13 = abs(dis1) / (abs(dis1) + abs(dis3));
				double r23 = abs(dis2) / (abs(dis2) + abs(dis3));
				osg::Vec3d v13 = getLerp(v1, v3, r13);
				osg::Vec3d v23 = getLerp(v2, v3, r23);
				addToVertexArray(v3, v13, v23, NEGATIVE_SIDE);
				addToVertexArray(v1, v2, v23, POSITIVE_SIDE);
				addToVertexArray(v1, v23, v13, POSITIVE_SIDE);
				if (nAry_)
				{
					osg::Vec3d n13 = getLerp(nAry_->at(p1), nAry_->at(p3), r13);
					osg::Vec3d n23 = getLerp(nAry_->at(p2), nAry_->at(p3), r23);
					addToNormalArray(nAry_->at(p3), n13, n23, NEGATIVE_SIDE);
					addToNormalArray(nAry_->at(p1), nAry_->at(p2), n23, POSITIVE_SIDE);
					addToNormalArray(nAry_->at(p1), n23, n13, POSITIVE_SIDE);
				}
				if (cAry_)
				{
					osg::Vec4 c13 = getLerp(getColor(cAry_, p1), getColor(cAry_, p3), r13);
					osg::Vec4 c23 = getLerp(getColor(cAry_, p2), getColor(cAry_, p3), r23);
					addToColorArray(getColor(cAry_, p3), c13, c23, NEGATIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), getColor(cAry_, p2), c23, POSITIVE_SIDE);
					addToColorArray(getColor(cAry_, p1), c23, c13, POSITIVE_SIDE);
				}
				if (tAry_)
				{
					osg::Vec2 t13 = getLerp(tAry_->at(p1), tAry_->at(p3), r13);
					osg::Vec2 t23 = getLerp(tAry_->at(p2), tAry_->at(p3), r23);
					addToTextCoordArray(tAry_->at(p3), t13, t23, NEGATIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), tAry_->at(p2), t23, POSITIVE_SIDE);
					addToTextCoordArray(tAry_->at(p1), t23, t13, POSITIVE_SIDE);
				}
			}
		}
	}

private:
	void addToVertexArray(const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3, Side side)
	{
		if (side == POSITIVE_SIDE)
		{
			lvAry_->push_back(v1);
			lvAry_->push_back(v2);
			lvAry_->push_back(v3);
		}
		else
		{
			rvAry_->push_back(v1);
			rvAry_->push_back(v2);
			rvAry_->push_back(v3);
		}
	}

	void addToNormalArray(const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3, Side side)
	{
		if (side == POSITIVE_SIDE)
		{
			lnAry_->push_back(v1);
			lnAry_->push_back(v2);
			lnAry_->push_back(v3);
		}
		else
		{
			rnAry_->push_back(v1);
			rnAry_->push_back(v2);
			rnAry_->push_back(v3);
		}
	}

	osg::Vec4 getColor(const osg::Vec4Array* ca, unsigned int i)
	{
		if (!ca) return osg::Vec4(1, 1, 1, 1);
		if (ca->empty()) return osg::Vec4(1, 1, 1, 1);

		if (geom_->getColorBinding() != osg::Geometry::BIND_PER_VERTEX)
			return ca->at(0);
		else
			return ca->at(i);
		return osg::Vec4(1, 1, 1, 1);
	}

	osg::Vec3 getNormal(const osg::Vec3Array* na, unsigned int i)
	{
		if (!na) return osg::Vec3();
		if (na->empty()) return osg::Vec3();

		if (geom_->getNormalBinding() != osg::Geometry::BIND_PER_VERTEX)
			return na->at(0);
		else
			return na->at(i);
		return osg::Vec3();
	}

	void addToColorArray(const osg::Vec4d& v1, const osg::Vec4d& v2, const osg::Vec4d& v3, Side side)
	{
		if (side == POSITIVE_SIDE)
		{
			lcAry_->push_back(v1);
			lcAry_->push_back(v2);
			lcAry_->push_back(v3);
		}
		else
		{
			rcAry_->push_back(v1);
			rcAry_->push_back(v2);
			rcAry_->push_back(v3);
		}
	}

	void addToTextCoordArray(const osg::Vec2d& v1, const osg::Vec2d& v2, const osg::Vec2d& v3, Side side)
	{
		if (side == POSITIVE_SIDE)
		{
			ltAry_->push_back(v1);
			ltAry_->push_back(v2);
			ltAry_->push_back(v3);
		}
		else
		{
			rtAry_->push_back(v1);
			rtAry_->push_back(v2);
			rtAry_->push_back(v3);
		}
	}

	osg::Vec3d getLerp(const osg::Vec3d& ps, const osg::Vec3d& pe, double rate)
	{
		osg::Vec3d dir = pe - ps;
		double len = dir.length();
		dir.normalize();
		return ps + dir*len*rate;
	}

	osg::Vec4 getLerp(const osg::Vec4& ps, const osg::Vec4& pe, double rate)
	{
		osg::Vec4 dir = pe - ps;
		double len = dir.length();
		dir.normalize();
		return ps + dir*len*rate;
	}

	osg::Vec2 getLerp(const osg::Vec2& ps, const osg::Vec2& pe, double rate)
	{
		osg::Vec2 dir = pe - ps;
		double len = dir.length();
		dir.normalize();
		return ps + dir*len*rate;
	}

private:
	osg::ref_ptr<const osg::Vec3Array> vAry_;
	osg::ref_ptr<const osg::Vec3Array> nAry_;
	osg::ref_ptr<const osg::Vec2Array> tAry_;
	osg::ref_ptr<const osg::Vec4Array> cAry_;
	osg::ref_ptr<osg::Geometry> geom_;

	osg::ref_ptr<osg::Vec4Array> rcAry_, lcAry_;
	osg::ref_ptr<osg::Vec3Array> rvAry_, lvAry_;
	osg::ref_ptr<osg::Vec3Array> rnAry_, lnAry_;
	osg::ref_ptr<osg::Vec2Array> rtAry_, ltAry_;
	osg::Plane plane_;
};

class GeometryColl :public osg::NodeVisitor
{
public:
	GeometryColl(const osg::Matrixd& mat)
		:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		, _mat(mat) {}
	virtual void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;
			_geomVector.push_back(geom);
		}
	}
	typedef std::vector < osg::ref_ptr<osg::Geometry> >GeometryVector;
	const GeometryVector& getGeometrys() const { return _geomVector; }
private:
	GeometryVector _geomVector;
	osg::Matrixd _mat;
};

//
class Cutter : public osg::NodeVisitor
{
public:
	Cutter(const osg::Plane& plane)
		: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		, _plane(plane)
		, _negativeRes(0L)
		, _positiveRes(0L)
	{ }

	osg::Node* getPositive()
	{
		if (_positiveRes)
		{
			//osgUtil::Optimizer o;
			//o.optimize(_positiveRes.get(),
			//	osgUtil::Optimizer::INDEX_MESH |
			//	osgUtil::Optimizer::VERTEX_PRETRANSFORM |
			//	osgUtil::Optimizer::VERTEX_POSTTRANSFORM);

			//osgUtil::IndexMeshVisitor imv;
			//_positiveRes->accept(imv);

			//osgUtil::VertexCacheVisitor vcv;
			//_positiveRes->accept(vcv);

			//osgUtil::VertexCacheMissVisitor vcmv;
			//_positiveRes->accept(vcmv);
		}
		return _positiveRes;
	}

	osg::Node* getNegative()
	{
		if (_negativeRes)
		{
			//osgUtil::Optimizer o;
			//o.optimize(_negativeRes.get(),
			//	osgUtil::Optimizer::INDEX_MESH |
			//	osgUtil::Optimizer::VERTEX_PRETRANSFORM |
			//	osgUtil::Optimizer::VERTEX_POSTTRANSFORM);

			//osgUtil::IndexMeshVisitor imv;
			//_negativeRes->accept(imv);

			//osgUtil::VertexCacheVisitor vcv;
			//_negativeRes->accept(vcv);

			//osgUtil::VertexCacheMissVisitor vcmv;
			//_negativeRes->accept(vcmv);
		}
		return _negativeRes;
	}

	void reset()
	{
		_positiveRes = 0L;
		_negativeRes = 0L;
	}

	void apply(osg::Geode& geode)
	{
		int geom_num = geode.getNumDrawables();
		for (int i = 0; i < geom_num; ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			osg::MatrixList ml = geom->getWorldMatrices();
			osg::Matrixd matrix = getMatrix(ml);
			osg::Matrixd inversematrix = osg::Matrixd::inverse(matrix);

			osg::Plane plane = _plane;
			plane.transform(inversematrix);

			SliceOperator so;
			osg::ref_ptr<osg::Node> pg = so.sliceGeom(geom, plane);
			std::vector<osg::ref_ptr<osg::Geometry> > geomVector;
			if (pg)
			{
				GeometryColl geometrycoll(inversematrix);
				pg->accept(geometrycoll);
				for (unsigned int i = 0; i < geometrycoll.getGeometrys().size(); ++i)
				{
					geomVector.push_back(geometrycoll.getGeometrys()[i]);
				}
			}

			osg::TriangleIndexFunctor<TriangulateOperator> MyTriangleIndexFunctor;
			MyTriangleIndexFunctor.setCutPlane(plane);
			MyTriangleIndexFunctor.getGeomProperties(*geom);
			geom->accept(MyTriangleIndexFunctor);

			osg::ref_ptr<osg::Geometry> positiveGeom = MyTriangleIndexFunctor.getGeometry(TriangulateOperator::POSITIVE_SIDE);
			if (positiveGeom.valid())
			{
				osg::ref_ptr<osg::Geode> positiveGeode = new osg::Geode;
				positiveGeode->addDrawable(positiveGeom);
				for (unsigned int geomi = 0; geomi < geomVector.size(); ++geomi)
				{
					positiveGeode->addDrawable(geomVector[geomi]);
				}
				gwUtil::MeshConsolidator::run(*positiveGeode);
				osg::ref_ptr<osg::MatrixTransform> positiveMat = new osg::MatrixTransform;
				positiveMat->setMatrix(matrix);
				positiveMat->addChild(positiveGeode);

				if (!_positiveRes) _positiveRes = new osg::Group;
				_positiveRes->addChild(positiveMat);
			}
			osg::ref_ptr<osg::Geometry> negativeGeom = MyTriangleIndexFunctor.getGeometry(TriangulateOperator::NEGATIVE_SIDE);
			if (negativeGeom.valid())
			{
				osg::ref_ptr<osg::Geode> negativeGeode = new osg::Geode;
				negativeGeode->addDrawable(negativeGeom);
				for (unsigned int geomi = 0; geomi < geomVector.size(); ++geomi)
				{
					negativeGeode->addDrawable(geomVector[geomi]);
				}
				gwUtil::MeshConsolidator::run(*negativeGeode);
				osg::ref_ptr<osg::MatrixTransform> negativeMat = new osg::MatrixTransform;
				negativeMat->setMatrix(matrix);
				negativeMat->addChild(negativeGeode);

				if (!_negativeRes) _negativeRes = new osg::Group;
				_negativeRes->addChild(negativeMat);
			}
		}
	}

private:
	osg::Matrixd getMatrix(const osg::MatrixList& ml)
	{
		return ml.empty() ? osg::Matrixd::identity() : ml[0];
	}

private:
	osg::Plane _plane;
	osg::ref_ptr<osg::Group> _positiveRes;
	osg::ref_ptr<osg::Group> _negativeRes;
	osg::ref_ptr<osg::Node> _profileNode;
};

#endif // GEOLOGICALTOOL_CUTTER_H
