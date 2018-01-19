#include "SliceOperator.h"
#include "GeneratorHelp.h"

#include <gwGeologicalUtil/PlaneIntersector.h>
#include <gwGeologicalUtil/Random.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/graph_traits_Dual_Arrangement_2.h>
#include <CGAL/Arr_face_index_map.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Polygon_with_holes_2.h>


#include <climits>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/visitors.hpp>

#include <osg/Vec3>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/TriangleFunctor>
#include <osg/MatrixTransform>
#include <osg/TriangleIndexFunctor>
#include <osgUtil/Tessellator>
#include <osgUtil/TriStripVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgDB/WriteFile>
//#include <assert.h>

//#ifdef _DEBUG
//#pragma comment(lib, "CGAL-vc140-mt-gd-4.11.lib")
//#pragma comment(lib, "CGAL_Core-vc140-mt-gd-4.11.lib")
//#else
//#pragma comment(lib, "CGAL-vc140-mt-4.11.lib")
//#pragma comment(lib, "CGAL_Core-vc140-mt-4.11.lib")
//#endif // _DEBUG
//#pragma comment(lib, "libgmp-10.lib")
//#pragma comment(lib, "libmpfr-4.lib")

const bool showSlicePoint = 1;
const bool fillRenderMode = true;
//
namespace
{
	typedef CGAL::Exact_predicates_exact_constructions_kernel	 Kernel;
	typedef CGAL::Arr_segment_traits_2<Kernel>                   Traits_2;
	typedef CGAL::Arr_face_extended_dcel<Traits_2, unsigned int> Dcel;
	typedef CGAL::Arrangement_2<Traits_2, Dcel>                  Ex_arrangement;
	typedef Kernel::Point_2                                      Point_2;
	typedef Kernel::Segment_2                                    Segment_2;
	//typedef CGAL::Partition_traits_2<Kernel>                     PTraits_2;
	//typedef PTraits_2::Polygon_2                                 Polygon_2;
	//typedef std::list<Polygon_2>                                 Polygon_list;
	//typedef CGAL::Polygon_with_holes_2<Kernel>                   Polygon_with_holes;
	//typedef std::list<Polygon_with_holes>                        Pgn_with_holes_container;

	template<class Arrangement>
	osg::Vec3dArray* getCbbArray(typename Arrangement::Ccb_halfedge_const_circulator circ, const osg::Matrixd& mat)
	{
		typename Arrangement::Ccb_halfedge_const_circulator  curr = circ;
		typename Arrangement::Halfedge_const_handle          he;

		osg::ref_ptr<osg::Vec3dArray> loopVa = new osg::Vec3dArray;
		double sx = CGAL::to_double(curr->source()->point().x());
		double sy = CGAL::to_double(curr->source()->point().y());
		osg::Vec3d sp = osg::Vec3d(sx, sy, 0.0);
		loopVa->push_back(sp*mat);
		do
		{
			he = curr;
			double x = CGAL::to_double(he->target()->point().x());
			double y = CGAL::to_double(he->target()->point().y());
			osg::Vec3d p = osg::Vec3d(x, y, 0.0);
			loopVa->push_back(p*mat);
			++curr;
		} while (curr != circ);

		return loopVa.release();
	}

	bool check_inside(Point_2 pt, const std::vector<Point_2>& points, Kernel traits)
	{
		Point_2 *buffer = new Point_2[points.size()];
		if (!points.empty())
		{
			memcpy(buffer, &points[0], points.size() * sizeof(Point_2));
		}
		switch (CGAL::bounded_side_2(buffer, buffer + points.size(), pt, traits))
		{
		case CGAL::ON_BOUNDED_SIDE:
			return true;
		case CGAL::ON_BOUNDARY:
			return true;
		case CGAL::ON_UNBOUNDED_SIDE:
			return false;
		default:
			return false;
		}
	}

	typedef std::vector<osg::ref_ptr<osg::Vec3dArray> > Vec3dArrays;
	struct DLoop : public osg::Referenced
	{
		DLoop() { _bb.init(); }
		osg::ref_ptr<osg::Vec3dArray> _va;
		Vec3dArrays _holes;
		void dirtyBound()
		{
			for (osg::Vec3dArray::const_iterator it = _va->begin(); it != _va->end(); ++it)
			{
				_bb.expandBy(*it);
			}
		}
		osg::BoundingBox _bb;
	};
	typedef std::vector<osg::ref_ptr<DLoop> > DLoops;

	//
	static bool isRepeat(const osg::Vec3dArray*va, const osg::Vec3d& p, double epsilon)
	{
		for (osg::Vec3dArray::const_iterator it = va->begin(); it != va->end(); ++it)
		{
			if (isVec3dEqual(*it, p, epsilon)) return true;
		}
		return false;
	}

	static bool isAlmostSameArray(const osg::Vec3dArray* va1, const osg::Vec3dArray* va2, double epsilon)
	{
		if (va1->size() != va2->size()) return false;
		osg::Vec3dArray::const_iterator it1;
		for (it1 = va1->begin(); it1 != va1->end(); ++it1)
		{
			if (!isRepeat(va2, *it1, epsilon)) return false;
		}
		return true;
	}

	static bool isContain(const osg::Vec3dArray* va1, const osg::Vec3dArray* va2, double epsilon)
	{
		osg::Vec3dArray::const_iterator it1;
		for (it1 = va1->begin(); it1 != va1->end(); ++it1)
		{
			if (!isRepeat(va2, *it1, epsilon)) return false;
		}
		return true;
	}

	template<typename ARRAY_TYPE = osg::Vec3dArray>
	static osg::Vec3d getNearestFarerThan(const ARRAY_TYPE* va, const osg::Vec3d& p, double dis)
	{
		double sumDis = 0.0;
		ARRAY_TYPE::const_iterator it;
		for (it = va->begin() + 1; it != va->end(); ++it)
		{
			sumDis += ((*it) - (*(it - 1))).length();
			if (sumDis >= dis) return *it;
		}
		return va->at(va->size() - 1);
	}

	osg::Geode* generatePoint(osg::Vec3Array* va, const osg::Vec4& color, float pointsize)
	{
		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
		for (unsigned int i = 0; i < va->size(); ++i)
		{
			ca->push_back(color);
		}
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setUseDisplayList(false);
		geom->setUseVertexBufferObjects(true);
		geom->setVertexArray(va);
		geom->setColorArray(ca, osg::Array::BIND_PER_VERTEX);
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, va->size()));

		osg::ref_ptr<osg::Point> point = new osg::Point;
		point->setSize(pointsize);
		geom->getOrCreateStateSet()->setAttributeAndModes(point);

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(geom);

		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		return geode.release();
	}

	osg::Geode* generatePoint(osg::Vec3Array* va, osg::Vec4Array* ca, float pointsize)
	{
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setUseDisplayList(false);
		geom->setUseVertexBufferObjects(true);
		geom->setVertexArray(va);
		geom->setColorArray(ca, osg::Array::BIND_PER_VERTEX);
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, va->size()));

		osg::ref_ptr<osg::Point> point = new osg::Point;
		point->setSize(pointsize);
		geom->getOrCreateStateSet()->setAttributeAndModes(point);

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(geom);

		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		return geode.release();
	}

	osg::Geode* generateLine(osg::Vec3Array* va, const osg::Vec4& color)
	{
		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
		for (unsigned int i = 0; i < va->size(); ++i)
		{
			ca->push_back(color);
		}
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setUseDisplayList(false);
		geom->setUseVertexBufferObjects(true);
		geom->setVertexArray(va);
		geom->setColorArray(ca, osg::Array::BIND_PER_VERTEX);
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, va->size()));

		osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth;
		lw->setWidth(5.0f);

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(geom);
		geode->getOrCreateStateSet()->setAttributeAndModes(lw);

		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		return geode.release();
	}

	//  [10/15/2017 ASUS]
	void generateLoops(const Vec3dArrayVector &vaVector, DLoops& loops, Vec3dArrays& holes, const osg::Matrixd& mat)
	{
		Ex_arrangement arr;
		std::vector<Segment_2> segments;

		Vec3dArrayVector::const_iterator it;

		unsigned int segmentNum = 0;
		for (it = vaVector.begin(); it != vaVector.end(); ++it)
			segmentNum += (*it)->size() - 1;
		segments.reserve(segmentNum);

		for (it = vaVector.begin(); it != vaVector.end(); ++it)
		{
			if ((*it)->size() < 2) continue;

			osg::Vec3dArray* va = *it;
			for (int i = 0; i < (int)va->size() - 1; ++i)
			{
				osg::Vec3d v0 = va->at(i)*mat;
				osg::Vec3d v1 = va->at(i + 1)*mat;
				Point_2 p0(v0[0], v0[1]);
				Point_2 p1(v1[0], v1[1]);
				if (p0 == p1) continue;//不允许重复的点;
				segments.push_back(Segment_2(p0, p1));
			}
		}
		CGAL::insert_curves(arr, segments.begin(), segments.end());
		segments.clear();

		if (arr.number_of_faces() < 1) return;

		osg::Matrixd inverseMat = osg::Matrixd::inverse(mat);

		Ex_arrangement::Face_const_iterator  fit;
		for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
		{
			if (fit == arr.unbounded_face()) continue;

			//得到外环;
			osg::ref_ptr<osg::Vec3dArray> loopVa = getCbbArray<Ex_arrangement>(fit->outer_ccb(), inverseMat);
			if (loopVa->empty()) continue;

			osg::ref_ptr<DLoop> loop = new DLoop;
			loop->_va = loopVa;
			//得到孔洞;
			if (fit->number_of_holes() > 0)
			{
				Ex_arrangement::Hole_const_iterator hi;
				for (hi = fit->holes_begin(); hi != fit->holes_end(); ++hi)
				{
					osg::ref_ptr<osg::Vec3dArray> holeVa = getCbbArray<Ex_arrangement>(*hi, inverseMat);
					if (holeVa->empty()) continue;
					loop->_holes.push_back(holeVa);
					holes.push_back(holeVa);
				}
			}
			loop->dirtyBound();
			loops.push_back(loop);
		}
	}

	//得到嵌套层数;
	unsigned int getNestNum(const DLoops &loops, const osg::Vec3dArray* va)
	{
		unsigned int num = 0;
		if (!va || loops.empty()) return num;

		const osg::Vec3dArray* tva = va;

		bool isFinish = false;
		while (!isFinish)
		{
			bool findHole = false;
			for (DLoops::const_iterator it = loops.begin(); it != loops.end(); ++it)
			{
				Vec3dArrays::const_iterator hit;
				for (hit = (*it)->_holes.begin(); hit != (*it)->_holes.end(); ++hit)
				{
					if (isAlmostSameArray(tva, *hit, GEO_EPSILOND))
					{
						findHole = true;
						tva = (*it)->_va;
						num++;
						break;
					}
				}
			}
			if (!findHole) isFinish = true;
		}
		return num;
	}

	bool isHole(const DLoops &loops, const osg::Vec3dArray* va)
	{
		return getNestNum(loops, va) % 2 == 1;
	}

	osg::Node* generateProfileNode(const DLoops &loops, const Vec3dArrays& holes, osg::Geometry* geom, const osg::Plane &sliceplane)
	{
		osg::ref_ptr<osg::Geode> profileGeode = new osg::Geode;
		for (DLoops::const_iterator it = loops.begin(); it != loops.end(); ++it)
		{
			DLoop* loop = *it;
			if (!loop) continue;

			//是孔洞则忽略;
			if (isHole(loops, loop->_va))
			{
				continue;
			}

			//不是孔洞就绘制;
			osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
			osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array;
			na->push_back(sliceplane.getNormal());
			osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
			ca->push_back(getGeometryColor(geom));

			osg::ref_ptr<osg::Geometry> profileGeom = new osg::Geometry;
			profileGeom->setVertexArray(va);
			profileGeom->setNormalArray(na, osg::Array::BIND_OVERALL);
			profileGeom->setColorArray(ca, osg::Array::BIND_OVERALL);

			unsigned int vaSize = loop->_va->size();
			for (Vec3dArrays::const_iterator holeIt = loop->_holes.begin(); holeIt != loop->_holes.end(); ++holeIt)
			{
				vaSize += holeIt->get()->size();
			}
			va->reserve(vaSize);

			for (osg::Vec3dArray::const_iterator vaIt = loop->_va->begin(); vaIt != loop->_va->end(); ++vaIt)
			{
				va->push_back(*vaIt);
			}
			profileGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, va->size()));

			unsigned int preIndex = va->size();
			for (Vec3dArrays::const_iterator holeIt = loop->_holes.begin(); holeIt != loop->_holes.end(); ++holeIt)
			{
				for (osg::Vec3dArray::const_iterator holeVaIt = (*holeIt)->begin(); holeVaIt != (*holeIt)->end(); ++holeVaIt)
				{
					va->push_back(*holeVaIt);
				}
				profileGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, preIndex, (*holeIt)->size()));
				preIndex += (*holeIt)->size();
			}

			osgUtil::Tessellator tscx;
			tscx.setTessellationNormal((sliceplane.getNormal()));
			tscx.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
			tscx.setBoundaryOnly(false);
			tscx.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
			tscx.retessellatePolygons(*profileGeom);

			//osg::TriangleFunctor<TrangleOperator> tf;
			//profileGeom->accept(tf);
			//if (!tf._isFind)
			//{
			//	continue;
			//}
			profileGeode->addDrawable(profileGeom);
		}

		if (fillRenderMode)//test
		{
			osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
			pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
			profileGeode->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
			profileGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		}

		return profileGeode.release();
	}

	osg::Node* generateSliceInformationNode(const Vec3dArrayVector &vaVector, /*const*/ gwUtil::Random &_random, const DLoops &loops)
	{
		osg::ref_ptr<osg::Group> mt = new osg::Group;
		{//slice point
			for (unsigned int i = 0; i < vaVector.size(); ++i)
			{
				osg::Vec4 color = osg::Vec4(_random.next(), _random.next(), _random.next(), 1.0);

				osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
				osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
				for (unsigned int j = 0; j < vaVector[i]->size(); ++j)
				{
					va->push_back(vaVector[i]->at(j));
					if (j == 0 || j == vaVector[i]->size() - 1)
					{
						ca->push_back(osg::Vec4(1, 1, 0, 1));
					}
					else
					{
						ca->push_back(/*osg::Vec4(1, 1, 0, 1)*/color);
					}
				}
				mt->addChild(generatePoint(va, ca/*osg::Vec4(0, 0, 1, 1)*/, 6.0f));
				mt->addChild(generateLine(va, color));
			}
		}
		{
			for (DLoops::const_iterator it = loops.begin(); it != loops.end(); ++it)
			{
				DLoop* loop = *it;
				if (!loop) continue;

				osg::Vec4 color = osg::Vec4(0, 1, 0, 1);

				for (Vec3dArrays::const_iterator holeIt = loop->_holes.begin(); holeIt != loop->_holes.end(); ++holeIt)
				{
					osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
					for (osg::Vec3dArray::const_iterator holeVaIt = (*holeIt)->begin(); holeVaIt != (*holeIt)->end(); ++holeVaIt)
					{
						va->push_back(*holeVaIt);
					}
					mt->addChild(generatePoint(va,/* osg::Vec4(0, 1, 0, 1)*/color, 9.50f));
				}
			}
		}
		return mt.release();
	}

	bool isSubsetOf(const osg::Vec3dArray& va, const osg::Vec3dArray& ova)
	{
		if (va.empty() || ova.empty()) return false;
		if (va.size() > ova.size()) return false;

		osg::Vec3dArray::const_iterator sit1 = va.begin();
		osg::Vec3dArray::const_iterator sit2 = std::find(ova.begin(), ova.end(), *sit1);
		if (sit2 == ova.end())
		{
			return false;
		}

		if (ova.end() - sit2 < va.size() /*&& sit2 - ova.begin() < va.size()*/)
		{
			return false;
		}

		//if (ova.end() - sit2 > va.size())
		{
			bool issub1 = true;
			osg::Vec3dArray::const_iterator it1, it2;
			for (it1 = sit1, it2 = sit2; it1 != va.end(); ++it1, ++it2)
			{
				if (!isVec3dEqual((*it1), *it2, GEO_EPSILOND))
				{
					issub1 = false;
					break;
				}
			}
			if (issub1) return true;
		}
		//if (sit2 - ova.begin() > va.size())
		//{
		//	bool issub2 = true;
		//	osg::Vec3dArray::const_iterator it1, it2;
		//	for (it1 = sit1, it2 = sit2; it1 != va.end(); ++it1, --it2)
		//	{
		//		if (!isVec3dEqual((*it1), *it2, GEO_EPSILOND))
		//		{
		//			issub2 = false;
		//			break;
		//		}
		//	}
		//	if (issub2) return true;
		//}
		return false;
	}

	//找出相内切的环;
	void reorganizeLoops(DLoops &loops)
	{
		for (DLoops::iterator it = loops.begin(); it != loops.end(); /*++it*/)
		{
			DLoop* loop = *it;

			DLoop* outerLoop = 0L;
			for (DLoops::const_iterator ait = loops.begin(); ait != loops.end(); ++ait)
			{
				DLoop* loop1 = *ait;
				if (loop == loop1)continue;
				if (isContain(loop->_va, loop1->_va, GEO_EPSILOND))
				{
					outerLoop = loop1;
					break;
				}
			}
			if (outerLoop)
			{
				//std::reverse(loop->_va->begin(), loop->_va->end());
				//outerLoop->_holes.push_back(loop->_va);
				it = loops.erase(it);
				OSG_NOTICE << "find a circle" << std::endl;
			}
			//else
			{
				++it;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
SliceOperator::SliceOperator(bool isOnEarth) :_isOnEarth(isOnEarth) {}
SliceOperator::~SliceOperator() {}

osg::Vec2Array* SliceOperator::generateTextureCoord(const osg::Vec3d& normal, osg::ref_ptr<osg::Vec3Array> va)
{
	osg::Vec3d nn = normalize(normal);
	osg::Vec3d p1 = getNearestFarerThan<osg::Vec3Array>(va, va->at(0), 10.0);

	osg::Vec3d normal_s = normalize(p1 - va->at(0));
	osg::Vec3d normal_t = normalize(nn^normal_s);

	osg::Plane plane_s(normal_s, va->at(0));
	osg::Plane plane_t(normal_t, va->at(0));
	osg::ref_ptr<osg::Vec2Array> ta = new osg::Vec2Array();
	ta->reserve(va->size());
	for (osg::Vec3Array::const_iterator it = va->begin(); it != va->end(); ++it)
	{
		double ts = plane_s.distance(*it);
		double ty = plane_t.distance(*it);
		ta->push_back(osg::Vec2(ts, ty));
	}
	return ta.release();
}

bool compairLoops(osg::ref_ptr<DLoop> loop1, osg::ref_ptr<DLoop> loop2)
{
	return loop1->_bb.radius() < loop2->_bb.radius();
}

osg::Node* SliceOperator::sliceGeom(osg::Geometry* geom, const osg::Plane& sliceplane)
{
	if (!geom) return 0L;

	Vec3dArrayVector vaVector;
	vaVector.clear();

	osg::ref_ptr<gwUtil::PlaneIntersector> intersector = new gwUtil::PlaneIntersector(sliceplane);
	intersector->setRecordHeightsAsAttributes(false);

	osgUtil::IntersectionVisitor intersectionVisitor;
	intersectionVisitor.reset();
	intersectionVisitor.setIntersector(intersector.get());
	geom->accept(intersectionVisitor);

	gwUtil::PlaneIntersector::Intersections& intersections = intersector->getIntersections();
	if (intersections.empty()) return 0L;

	gwUtil::PlaneIntersector::Intersections::iterator itr;
	for (itr = intersections.begin(); itr != intersections.end(); ++itr)
	{
		gwUtil::PlaneIntersector::Intersection& intersection = *itr;
		osg::ref_ptr<osg::Vec3dArray> vva = new osg::Vec3dArray;
		for (auto pitr = intersection.polyline.begin(); pitr != intersection.polyline.end(); ++pitr)
		{
			osg::Vec3d p = intersection.matrix.valid() ? (*pitr) * (*intersection.matrix) : (*pitr);
			vva->push_back(p);
		}
		intersection.matrix = 0;
		//删除重复冗余的点;
		removeRepeatExt<osg::Vec3dArray>(vva);
		removeNextRepeatExt<osg::Vec3dArray>(vva);
		if (vva->empty()) continue;
		vaVector.push_back(vva);
	}

	const osg::Vec3d planenormal = sliceplane.getNormal();
	osg::Vec3d transformVec = planenormal*sliceplane.asVec4()[3];
	osg::Quat rot;
	rot.makeRotate(planenormal, osg::Vec3d(0, 0, 1));

	osg::Matrixd mat = osg::Matrixd::translate(transformVec)*osg::Matrixd::rotate(rot);

	DLoops loops;
	Vec3dArrays holes;
	generateLoops(vaVector, loops, holes, mat);
	std::sort(loops.begin(), loops.end(), compairLoops);
	reorganizeLoops(loops);

	osg::ref_ptr<osg::Node> profilenode = generateProfileNode(loops, holes, geom, sliceplane);

	osg::ref_ptr<osg::Group> resultGroup = new osg::Group;
	if (profilenode) resultGroup->addChild(profilenode);

	if (showSlicePoint)
	{
		resultGroup->addChild(generateSliceInformationNode(vaVector, _random, loops));
	}
	return resultGroup.release();
}

namespace
{
	struct MyGeometry :public osg::Referenced
	{
		MyGeometry(osg::Geometry* geom, const osg::Matrixd& mat)
			: _geom(geom), _mat(mat)
		{}
		osg::Matrixd _mat;
		osg::ref_ptr<osg::Geometry> _geom;
	};
	typedef std::vector<osg::ref_ptr<MyGeometry> > MyGeometryVector;

	struct GeometryCollector :public osg::NodeVisitor
	{
		GeometryCollector() :osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

		void reset() { _geoms.clear(); }
		void apply(osg::Geode& geode)
		{
			for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
			{
				osg::Drawable* da = geode.getDrawable(i);
				if (!da) continue;

				osg::Geometry* geom = da->asGeometry();
				if (!geom) continue;

				osg::MatrixList matrixlist = geom->getWorldMatrices();
				osg::Matrixd mat = getMatrix(matrixlist);
				_geoms.push_back(new MyGeometry(geom, mat));
			}
			traverse(geode);
		}

		const MyGeometryVector& getGeoms() const { return _geoms; }

	private:
		osg::Matrixd getMatrix(const osg::MatrixList& ml)
		{
			if (!ml.empty())
			{
				return ml[0];
			}
			return osg::Matrixd::identity();
		}

	private:
		MyGeometryVector _geoms;
	};
}

osg::Node* SliceOperator::slice(osg::Node * geologicalModel, const osg::Plane& slicePlane)
{
	osg::ref_ptr<osg::Group> profileroot = new osg::Group;

	GeometryCollector gc; gc.reset();
	geologicalModel->accept(gc);

	const MyGeometryVector& geoms = gc.getGeoms();
	for (MyGeometryVector::const_iterator it = geoms.begin(); it != geoms.end(); ++it)
	{
		osg::Geometry* geom = (*it)->_geom;
		if (!geom) continue;

		const osg::Matrixd& matrix = (*it)->_mat;

		osg::Plane splane = osg::Plane(slicePlane);
		splane.transform(osg::Matrixd::inverse(matrix));

		osg::Node* profilenode = sliceGeom(geom, splane);
		if (!profilenode) continue;

		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		mt->setMatrix(matrix);
		mt->addChild(profilenode);

		profileroot->addChild(mt);
	}
	return profileroot.release();
}