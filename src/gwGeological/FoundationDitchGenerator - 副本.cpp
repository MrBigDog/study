#include "FoundationDitchGenerator.h"
#include "SliceOperator.h"
#include "Cutter.h"
#include "GeneratorHelp.h"
#include "clipper.hpp"

#include <osg/PolygonMode>
#include <osg/Stencil>
#include <osg/TriangleIndexFunctor>
#include <osgUtil/Tessellator>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;
typedef std::vector<Polygon_with_holes_2>                 Pwh_vector_2;
typedef std::vector<Point_2>                              PointVec;

namespace
{
	class TrangleVisitor
	{
	public:
		TrangleVisitor() :_newVa(new osg::Vec3Array)
		{}

		inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3)
		{
			if (!_va) return;

			const osg::Vec3d& v1 = _va->at(p1);
			const osg::Vec3d& v2 = _va->at(p2);
			const osg::Vec3d& v3 = _va->at(p3);

			if (isVec3dEqual(v1, v2, FLT_EPSILON) ||
				isVec3dEqual(v1, v3, FLT_EPSILON) ||
				isVec3dEqual(v2, v3, FLT_EPSILON))
			{
				return;
			}

			Polygon_2 triPolygon;
			triPolygon.push_back(Point_2(v1.x(), v1.y()));
			triPolygon.push_back(Point_2(v2.x(), v2.y()));
			triPolygon.push_back(Point_2(v3.x(), v3.y()));

			if (_clipPolygon.bbox().xmax() < triPolygon.bbox().xmin() ||
				_clipPolygon.bbox().xmin() > triPolygon.bbox().xmax() ||
				_clipPolygon.bbox().ymax() < triPolygon.bbox().ymin() ||
				_clipPolygon.bbox().ymin() > triPolygon.bbox().ymax())
			{
				return;
			}

			if (!CGAL::do_intersect(_clipPolygon, triPolygon))
			{
				return;
			}

			Pwh_list_2 intR;
			CGAL::intersection(_clipPolygon, triPolygon, std::back_inserter(intR));

			Pwh_list_2::const_iterator it;
			for (it = intR.begin(); it != intR.end(); ++it)
			{
				int bsize = (*it).outer_boundary().size();
				std::cout << bsize << std::endl;
				_resultPolygons.push_back(*it);
			}
		}

		void setClipPolygon(const Polygon_2& polygon)
		{
			_clipPolygon = polygon;
		}

		void setGeom(osg::Geometry* geom)
		{
			_geom = geom;
			_va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		}

		void join()
		{
			if (_resultPolygons.empty()) return;

			_resPolygon = *_resultPolygons.begin();
			for (Pwh_vector_2::const_iterator it = _resultPolygons.begin() + 1; it != _resultPolygons.end(); ++it)
			{
				Polygon_with_holes_2 polygon;
				if (CGAL::join(_resPolygon, *it, polygon))
				{
					_resPolygon = polygon;
				}
			}
		}

		const Polygon_with_holes_2& getResultPolygon() const
		{
			return _resPolygon;
		}

	private:
		Polygon_2 _clipPolygon;
		osg::Vec3Array* _va;
		osg::Geometry* _geom;
		osg::ref_ptr<osg::Vec3Array> _newVa;

		Polygon_with_holes_2 _resPolygon;
		Pwh_vector_2 _resultPolygons;
	};

	template<typename ARRAY_TYPE>
	static void removeRepeat(ARRAY_TYPE* va, double epsilon = FLT_EPSILON)
	{
		ARRAY_TYPE::iterator it, it1;
		for (it = ++va->begin(); it != va->end();)
		{
			bool isRepeat = false;
			for (it1 = va->begin(); it1 != it; ++it1)
			{
				if (isVec3dEqual(*it, *it1, epsilon))
				{
					isRepeat = true;
					break;
				}
			}
			if (isRepeat) it = va->erase(it);
			else it++;
		}
	}

	struct PolygonCliper :public osg::NodeVisitor
	{
		PolygonCliper(const osg::Vec3d& normal, double h, osg::Vec3Array* va)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _normal(normal), _h(h), _clipVa(va)
		{
			removeRepeat(_clipVa);
		}

		void apply(osg::Geode& polygonGeode)
		{
			osg::MatrixList ml = polygonGeode.getWorldMatrices();
			osg::Matrixd mat = ml[0];
			osg::Matrixd inverseMat = osg::Matrixd::inverse(mat);

			Polygon_2 clipPolygon;
			for (osg::Vec3Array::iterator it = _clipVa->begin(); it != _clipVa->end(); ++it)
			{
				osg::Vec3d v = (*it)*inverseMat;
				clipPolygon.push_back(Point_2(v.x(), v.y()));
			}
			_triFunctor.setClipPolygon(clipPolygon);

			//
			for (unsigned int i = 0; i < polygonGeode.getNumDrawables(); ++i)
			{
				osg::Geometry* geom = polygonGeode.getDrawable(i)->asGeometry();
				if (!geom) continue;

				osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
				if (!va) continue;

				osg::ref_ptr<osg::Vec4Array> ca = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
				osg::Vec4 color = (ca && !ca->empty()) ? ca->at(0) : osg::Vec4(1, 1, 1, 1);


				//Polygon_2 polygon;
				//for (osg::Vec3Array::const_iterator it = va->begin(); it != va->end(); ++it)
				//{
				//	polygon.push_back(Point_2((*it)[0], (*it)[1]));
				//}

				_triFunctor.setGeom(geom);
				geom->accept(_triFunctor);
				_triFunctor.join();

				const Polygon_with_holes_2& poy = _triFunctor.getResultPolygon();

				//ClipperLib::Path path; path.resize(va->size());
				//for (unsigned int j = 0; j < va->size(); ++j)
				//{
				//	path[j].X = va->at(j)[0];
				//	path[j].Y = va->at(j)[1];
				//}

				//ClipperLib::Clipper cliper;
				//cliper.AddPath(_clipPath, ClipperLib::ptClip, true);
				//cliper.AddPath(path, ClipperLib::ptSubject, true);

				//ClipperLib::Paths sol;
				//cliper.Execute(ClipperLib::ctIntersection, sol, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
				//for (ClipperLib::Paths::const_iterator it = sol.begin(); it != sol.end(); ++it)
				//{
				//	osg::ref_ptr<osg::Vec3Array> subva = new osg::Vec3Array;
				//	osg::ref_ptr<osg::Vec4Array> subca = new osg::Vec4Array;
				//	for (unsigned int k = 0; k < (*it).size(); ++k)
				//	{
				//		subva->push_back(_rot*osg::Vec3d((double)(*it)[k].X, (double)(*it)[k].Y, _h));
				//		subca->push_back(color);
				//	}
				//	osg::ref_ptr<osg::Geometry> subGeom = new osg::Geometry;
				//	subGeom->setStateSet(geom->getStateSet());
				//	subGeom->setUserData(geom->getUserData());
				//	subGeom->setVertexArray(subva);
				//	subGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, subva->size()));

				//	osgUtil::Tessellator tscx;
				//	tscx.setTessellationNormal(_normal);
				//	tscx.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
				//	tscx.setBoundaryOnly(false);
				//	tscx.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
				//	tscx.retessellatePolygons(*subGeom);

				//	_result->addDrawable(subGeom);
				//}
			}
		}

		//osg::Node* getResult() { return _result.release(); }

	private:
		double _h;
		//ClipperLib::Path _clipPath;
		osg::Vec3d _normal;
		//osg::Quat _rot;
		//osg::ref_ptr<osg::Geode> _result;
		osg::Vec3Array* _clipVa;
		osg::TriangleIndexFunctor<TrangleVisitor> _triFunctor;
	};
}


void setMaskState(osg::Node* node)
{
	osg::Stencil* stencil = new osg::Stencil;
	stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0u);
	stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);

	node->getOrCreateStateSet()->setAttributeAndModes(stencil, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	//node->getOrCreateStateSet()->setAttribute(new osg::ColorMask(false, false, false, false), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	node->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
}

osg::Node * FoundationDitchGenerator::operator()(osg::Node * geologicalGroup, const osg::Vec3Array * vva)
{
	if (vva->size() < 3 || !geologicalGroup) return 0L;
	//if (geologicalGroup->getNumChildren() < 1) return 0L;

	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
	va->reserve(vva->size() + 1);
	for (osg::Vec3Array::const_iterator it = vva->begin(); it != vva->end(); ++it)
	{
		va->push_back(*it);
	}
	removeRepeatExt<osg::Vec3Array>(va);
	va->push_back(vva->at(0));

	if (va->size() < 3) return 0L;

	SliceOperator slice(_isOnEarth);
	osg::ref_ptr<osg::Group> sideGroup = new osg::Group;
	setMaskState(sideGroup);
	for (osg::Vec3Array::const_iterator it = va->begin() + 1; it != va->end(); ++it)
	{
		osg::Vec3d dir = normalize(*it - *(it - 1));
		osg::Vec3d updir = _isOnEarth ? normalize(*(it - 1)) : osg::Vec3d(0, 0, 1);

		osg::Plane s(dir, *(it - 1));
		osg::Plane e(-dir, *it);
		osg::Plane p(*(it - 1), *it, *(it - 1) + updir);

		osg::Polytope pt;
		pt.add(s);
		pt.add(e);

		osg::ref_ptr<osg::Node> profileNode = slice.slice(geologicalGroup, p);
		if (!profileNode)
		{
			continue;
		}

		Cutter cut(pt);
		profileNode->accept(cut);

		PlaneGeometryOptimizer pgo(*it, p.getNormal(), dir);
		profileNode->accept(pgo);

		sideGroup->addChild(profileNode);
	}

	osg::Vec3d polyCenter = getCenter<osg::Vec3Array, osg::Vec3d>(va);
	osg::Plane bottomPlane(_isOnEarth ? normalize(polyCenter) : osg::Vec3d(0, 0, 1), polyCenter);

	//bottom
	osg::ref_ptr<osg::Group> profileGroup = new osg::Group;
	profileGroup->addChild(sideGroup);
	osg::ref_ptr<osg::Node> bottomNode = slice.slice(geologicalGroup, bottomPlane);
	if (bottomNode)
	{
		PolygonCliper pc(osg::Vec3d(0, 0, 1), va->at(0)[2], va);
		bottomNode->accept(pc);
		profileGroup->addChild(/*pc.getResult()*/ bottomNode);
	}

	//{//test
	//	osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	//	pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
	//	profileGroup->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
	//	profileGroup->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	//}

	return profileGroup.release();
}

void FoundationDitchGenerator::generateSlicePlanes(const osg::Vec3Array * vva, bool isOnEarth, SlicePLaneVector & out_slice_planes)
{
	if (!vva) return;
	if (vva->size() < 3) return;

	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
	va->reserve(vva->size() + 1);
	for (osg::Vec3Array::const_iterator it = vva->begin(); it != vva->end(); ++it)
	{
		va->push_back(*it);
	}
	removeRepeatExt<osg::Vec3Array>(va);
	va->push_back(vva->at(0));

	if (va->size() < 3) return;

	for (osg::Vec3Array::const_iterator it = va->begin() + 1; it != va->end(); ++it)
	{
		osg::Vec3d dir = normalize(*it - *(it - 1));
		osg::Vec3d updir = isOnEarth ? normalize(*(it - 1)) : osg::Vec3d(0, 0, 1);

		osg::Plane s(dir, *(it - 1));
		osg::Plane e(-dir, *it);
		osg::Plane p(*(it - 1), *it, *(it - 1) + updir);

		osg::Polytope pt;
		pt.add(s);
		pt.add(e);
		out_slice_planes.push_back(SlicePlane(p, pt));
	}

	osg::Vec3d polyCenter = getCenter<osg::Vec3Array, osg::Vec3d>(va);
	osg::Plane bottomPlane(isOnEarth ? normalize(polyCenter) : osg::Vec3d(0, 0, 1), polyCenter);
	out_slice_planes.push_back(SlicePlane(bottomPlane, osg::Polytope()));
}