#include "SliceOperator.h"
#include "GeneratorHelp.h"

#include <gwGeologicalUtil/PlaneIntersector.h>
#include <gwGeologicalUtil/Random.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/graph_traits_Dual_Arrangement_2.h>
#include <CGAL/Arr_face_index_map.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Boolean_set_operations_2.h>

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

const bool showSlicePoint = 0;
const bool fillRenderMode = true;
//
namespace
{
	typedef CGAL::Exact_predicates_exact_constructions_kernel	 Kernel;
	//typedef CGAL::Arr_segment_traits_2<Kernel>                   Traits_2;
	typedef CGAL::Arr_non_caching_segment_traits_2<Kernel>                   Traits_2;
	typedef CGAL::Arr_face_extended_dcel<Traits_2, unsigned int> Dcel;
	typedef CGAL::Arrangement_2<Traits_2, Dcel>                  Ex_arrangement;
	typedef Kernel::Point_2                                      Point_2;
	typedef Kernel::Segment_2                                    Segment_2;
	typedef Ex_arrangement::Halfedge_handle                      Halfedge_handle;
	typedef CGAL::Polygon_2<Kernel>                             Polygon_2;
	//typedef CGAL::Polygon_with_holes_2<Kernel>                  Polygon_with_holes_2;
	//typedef std::list<Polygon_with_holes_2>                     Pwh_list_2;

	typedef std::vector<Point_2>                                 PointVec;
	typedef std::vector<Polygon_2>                                PolygonVec;

	template<class Arrangement>
	void getCbbArray(typename Arrangement::Ccb_halfedge_const_circulator circ, Polygon_2& polygon)
	{
		typename Arrangement::Ccb_halfedge_const_circulator  curr = circ;
		typename Arrangement::Halfedge_const_handle          he;

		unsigned int pnum = 0;
		do {
			he = curr;
			if (he->face() != he->twin()->face()) pnum++;
			++curr;
		} while (curr != circ);

		PointVec points; points.reserve(pnum);
		do
		{
			he = curr;
			if (he->face() != he->twin()->face())
			{
				points.push_back(he->target()->point());
				//polygon.push_back(he->target()->point());
			}
			++curr;
		} while (curr != circ);
		polygon.insert(polygon.vertices_begin(), points.begin(), points.end());
	}

	bool isInHole(Point_2 pt, const Polygon_2& hole)
	{
		switch (hole.bounded_side(pt))
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

	static bool isInHole(const Polygon_2& va1, const Polygon_2& hole)
	{
		if (va1.size() > hole.size()) return false;
		for (unsigned int i = 0; i < va1.size(); ++i)
		{
			if (!isInHole(va1.vertex(i), hole)) return false;
		}
		return true;
	}

	typedef std::vector<osg::ref_ptr<osg::Vec3dArray> > Vec3dArrays;
	struct DLoop : public osg::Referenced
	{
		DLoop() { }
		Polygon_2 _polygon;
		PolygonVec _holes;
	};
	typedef std::vector<osg::ref_ptr<DLoop> > DLoops;

	////
	//static bool isRepeat(const osg::Vec3dArray*va, const osg::Vec3d& p, double epsilon)
	//{
	//	for (osg::Vec3dArray::const_iterator it = va->begin(); it != va->end(); ++it)
	//	{
	//		if (isVec3dEqual(*it, p, epsilon)) return true;
	//	}
	//	return false;
	//}

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
		ca->push_back(color);

		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setUseDisplayList(false);
		geom->setUseVertexBufferObjects(true);
		geom->setVertexArray(va);
		geom->setColorArray(ca, osg::Array::BIND_OVERALL);
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
		ca->push_back(color);

		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setUseDisplayList(false);
		geom->setUseVertexBufferObjects(true);
		geom->setVertexArray(va);
		geom->setColorArray(ca, osg::Array::BIND_OVERALL);
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
		CGAL::insert_non_intersecting_curves(arr, segments.begin(), segments.end());
		//CGAL::insert_non_intersecting_curves()
		segments.clear();

		if (arr.number_of_faces() < 1) return;

		Ex_arrangement::Face_const_iterator  fit;
		for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
		{
			if (fit == arr.unbounded_face()) continue;

			//得到外环;
			osg::ref_ptr<DLoop> loop = new DLoop;
			getCbbArray<Ex_arrangement>(fit->outer_ccb(), loop->_polygon);

			//得到孔洞;
			if (fit->number_of_holes() > 0)
			{
				Ex_arrangement::Hole_const_iterator hi;
				for (hi = fit->holes_begin(); hi != fit->holes_end(); ++hi)
				{
					Polygon_2 holeP;
					getCbbArray<Ex_arrangement>(*hi, holeP);
					if (holeP.size() > 0)
					{
						loop->_holes.push_back(holeP);
					}
				}
			}
			loops.push_back(loop);
		}
	}

	//得到嵌套层数;
	unsigned int getNestNum(const DLoops &loops, const Polygon_2& p)
	{
		unsigned int num = 0;
		if (loops.empty()) return num;

		const Polygon_2* tp = &p;

		bool isFinish = false;
		while (!isFinish)
		{
			bool findHole = false;
			for (DLoops::const_iterator it = loops.begin(); it != loops.end(); ++it)
			{
				PolygonVec::const_iterator hit;
				for (hit = (*it)->_holes.begin(); hit != (*it)->_holes.end(); ++hit)
				{
					if (isInHole(*tp, *hit))
					{
						findHole = true;
						tp = &((*it)->_polygon);
						num++;
						break;
					}
				}
			}
			if (!findHole) isFinish = true;
		}
		if (num > 1)
		{
			//OSG_NOTICE << num << std::endl;
		}
		return num;
	}

	bool isHole(const DLoops &loops, const Polygon_2& va)
	{
		return getNestNum(loops, va) % 2 == 1;
	}

	osg::Node* generateProfileNode(const DLoops &loops, const Vec3dArrays& holes, osg::Geometry* geom, const osg::Plane &sliceplane, const osg::Matrixd& mat)
	{
		osg::ref_ptr<osg::Group> resultRoot = new osg::Group;
		for (DLoops::const_iterator it = loops.begin(); it != loops.end(); ++it)
		{
			DLoop* loop = *it;
			if (!loop) continue;

			//是孔洞则忽略;
			if (isHole(loops, loop->_polygon))
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
			profileGeom->setUseDisplayList(false);
			profileGeom->setUseVertexBufferObjects(true);
			profileGeom->setVertexArray(va);
			profileGeom->setNormalArray(na, osg::Array::BIND_OVERALL);
			profileGeom->setColorArray(ca, osg::Array::BIND_OVERALL);

			unsigned int vaSize = loop->_polygon.size();
			for (PolygonVec::const_iterator holeIt = loop->_holes.begin(); holeIt != loop->_holes.end(); ++holeIt)
				vaSize += holeIt->size();
			va->reserve(vaSize);

			for (Polygon_2::Vertex_const_iterator vaIt = loop->_polygon.vertices_begin(); vaIt != loop->_polygon.vertices_end(); ++vaIt)
			{
				osg::Vec3d v = osg::Vec3d(CGAL::to_double(vaIt->x()), CGAL::to_double(vaIt->y()), 0.0)*mat;
				va->push_back(v);
			}
			profileGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, va->size()));

			unsigned int preIndex = va->size();
			for (PolygonVec::const_iterator holeIt = loop->_holes.begin(); holeIt != loop->_holes.end(); ++holeIt)
			{
				for (Polygon_2::Vertex_const_iterator holeVaIt = holeIt->vertices_begin(); holeVaIt != holeIt->vertices_end(); ++holeVaIt)
				{
					osg::Vec3d v = osg::Vec3d(CGAL::to_double(holeVaIt->x()), CGAL::to_double(holeVaIt->y()), 0.0)*mat;
					va->push_back(v);
				}
				profileGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, preIndex, holeIt->size()));
				preIndex += holeIt->size();
			}

			osgUtil::Tessellator tscx;
			tscx.setTessellationNormal((sliceplane.getNormal()));
			tscx.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
			tscx.setBoundaryOnly(false);
			tscx.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
			tscx.retessellatePolygons(*profileGeom);

			osg::ref_ptr<osg::Geode> profileGeode = new osg::Geode;
			profileGeode->addDrawable(profileGeom);

			resultRoot->addChild(profileGeode);
		}

		if (fillRenderMode)//test
		{
			osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
			pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
			resultRoot->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
			resultRoot->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		}
		return resultRoot.release();
	}

	osg::Node* generateSliceInformationNode(const Vec3dArrayVector &vaVector, /*const*/ gwUtil::Random &_random, const DLoops &loops, const osg::Matrixd& mat)
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

				for (PolygonVec::const_iterator holeIt = loop->_holes.begin(); holeIt != loop->_holes.end(); ++holeIt)
				{
					osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
					for (Polygon_2::Vertex_const_iterator holeVaIt = holeIt->vertices_begin(); holeVaIt != holeIt->vertices_end(); ++holeVaIt)
					{
						osg::Vec3d v = osg::Vec3d(CGAL::to_double(holeVaIt->x()), CGAL::to_double(holeVaIt->y()), 0.0)*mat;
						va->push_back(v);
					}
					mt->addChild(generatePoint(va,/* osg::Vec4(0, 1, 0, 1)*/color, 9.50f));
				}
			}
		}
		return mt.release();
	}

	//bool isSubsetOf(const osg::Vec3dArray& va, const osg::Vec3dArray& ova, unsigned int& s, unsigned int& e)
	//{
	//	if (va.empty() || ova.empty()) return false;
	//	if (va.size() > ova.size()) return false;
	//	for (osg::Vec3dArray::const_iterator oit = ova.begin(); oit != ova.end(); ++oit)
	//	{
	//		if (!isVec3dEqual(*oit, *va.begin(), GEO_EPSILOND)) continue;
	//		//if (ova.end() - oit < va.size()) return false;

	//		bool isSubset = true;
	//		osg::Vec3dArray::const_iterator it, it1;
	//		for (it = va.begin(), it1 = oit; it != va.end(); ++it, ++it1)
	//		{
	//			if (it1 == ova.end()) it1 = ova.begin();
	//			if (!isVec3dEqual(*it, *it1, GEO_EPSILOND))
	//			{
	//				isSubset = false; break;
	//			}
	//		}
	//		if (isSubset)
	//		{
	//			s = oit - ova.begin();
	//			e = s + va.size();
	//			return true;
	//		}
	//	}
	//	return false;
	//}

	bool isOnEdges(const osg::Vec3d& v, const Vec3dArrayVector& edges)
	{
		for (Vec3dArrayVector::const_iterator it = edges.begin(); it != edges.end(); ++it)
		{
			for (osg::Vec3dArray::const_iterator vait = (*it)->begin(); vait != (*it)->end(); ++vait)
			{
				if (isVec3dEqual(v, *vait, GEO_EPSILOND))
				{
					return true;
				}
			}
		}
		return false;
	}

	static bool isHoleOf(const Polygon_2& p1, const Polygon_2& p2, const Vec3dArrayVector& edges, PointVec& pointOnEdge, PointVec& points, const osg::Matrixd& mat)
	{
		for (unsigned int i = 0; i < p1.size(); ++i)
		{
			const Point_2& point = p1.vertex(i);
			CGAL::Bounded_side bs = p2.bounded_side(p1.vertex(i));
			if (bs != CGAL::ON_BOUNDARY)
			{
				osg::Vec3d v = osg::Vec3d(CGAL::to_double(point.x()), CGAL::to_double(point.y()), 0.0)*mat;
				if (!isOnEdges(v, edges)) return false;
				pointOnEdge.push_back(point);
			}
			points.push_back(point);
		}
		return true;
	}

	//找出相内切的环;
	void reorganizeLoops(DLoops &loops, const Vec3dArrayVector& edges, const osg::Matrixd& mat)
	{
		for (DLoops::iterator it = loops.begin(); it != loops.end(); ++it)
		{
			DLoop* loop = *it;
			for (DLoops::const_iterator ait = loops.begin(); ait != loops.end(); ++ait)
			{
				DLoop* loop1 = *ait;
				if (loop == loop1) continue;

				PointVec pointOnEdeg, otherPoints;
				if (isHoleOf(loop->_polygon, loop1->_polygon, edges, pointOnEdeg, otherPoints, mat))
				{
					for (PointVec::const_iterator otherit = otherPoints.begin(); otherit != otherPoints.end(); ++otherit)
					{
						Polygon_2::Vertex_iterator iit = std::find(loop1->_polygon.vertices_begin(), loop1->_polygon.vertices_end(), *otherit);
						if (iit == loop1->_polygon.vertices_end()) continue;
						osg::Vec3d v = osg::Vec3d(CGAL::to_double(iit->x()), CGAL::to_double(iit->y()), 0.0)*mat;
						if (isOnEdges(v, edges)) continue;
						loop1->_polygon.erase(iit);
					}

					if (pointOnEdeg.size() > 2)
					{
						//OSG_NOTICE << "there: " << pointOnEdeg.size() << std::endl;
						Polygon_2::Vertex_iterator loop1vaIts = std::find(loop1->_polygon.vertices_begin(), loop1->_polygon.vertices_end(), pointOnEdeg.front());
						Polygon_2::Vertex_iterator loop1vaIte = std::find(loop1->_polygon.vertices_begin(), loop1->_polygon.vertices_end(), pointOnEdeg.back());
						if (loop1vaIte != loop1->_polygon.vertices_end() && loop1vaIts != loop1->_polygon.vertices_end())
						{
							if (loop1vaIts < loop1vaIte)
							{
								for (int edgei = (int)pointOnEdeg.size() - 1; edgei >= 0; --edgei)
								{
									loop1vaIte = loop1->_polygon.insert(loop1vaIte, pointOnEdeg[edgei]);
								}
							}
							else
							{
								for (int edgei = 0; edgei < (int)pointOnEdeg.size(); ++edgei)
								{
									loop1vaIts = loop1->_polygon.insert(loop1vaIts, pointOnEdeg[edgei]);
								}
							}
						}
					}
					loop1->_holes.push_back(loop->_polygon);
					break;
				}
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
	double x1 = loop1->_polygon.bbox().xmax() - loop1->_polygon.bbox().xmin();
	double y1 = loop1->_polygon.bbox().ymax() - loop1->_polygon.bbox().ymin();
	double x2 = loop2->_polygon.bbox().xmax() - loop2->_polygon.bbox().xmin();
	double y2 = loop2->_polygon.bbox().ymax() - loop2->_polygon.bbox().ymin();
	return x1*y1 < x2*y2;
}

osg::Node* SliceOperator::sliceGeom(osg::Geometry* geom, const osg::Plane& sliceplane)
{
	if (!geom) return 0L;

	Vec3dArrayVector vaVector;
	vaVector.clear();

	Vec3dArrayVector edgeVector;

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

		if (vva->empty()) continue;
		vaVector.push_back(vva);
		if ((int)intersector->getSize() <= itr - intersections.begin())
		{
			edgeVector.push_back(vva);
		}
	}

	const osg::Vec3d planenormal = sliceplane.getNormal();
	osg::Vec3d transformVec = planenormal*sliceplane.asVec4()[3];
	osg::Quat rot;
	rot.makeRotate(planenormal, osg::Vec3d(0, 0, 1));

	osg::Matrixd mat = osg::Matrixd::translate(transformVec)*osg::Matrixd::rotate(rot);

	osg::Matrixd inverseMat = osg::Matrixd::inverse(mat);

	DLoops loops;
	Vec3dArrays holes;
	generateLoops(vaVector, loops, holes, mat);
	std::sort(loops.begin(), loops.end(), compairLoops);
	reorganizeLoops(loops, edgeVector, inverseMat);

	osg::ref_ptr<osg::Node> profilenode = generateProfileNode(loops, holes, geom, sliceplane, inverseMat);

	osg::ref_ptr<osg::Group> resultGroup = new osg::Group;
	if (profilenode) resultGroup->addChild(profilenode);

	if (showSlicePoint)
	{
		resultGroup->addChild(generateSliceInformationNode(vaVector, _random, loops, inverseMat));
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

	//osgDB::writeNodeFile(*profileroot, "profileTest.osgb");
	return profileroot.release();
}