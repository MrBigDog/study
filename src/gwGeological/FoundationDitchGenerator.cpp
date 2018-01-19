#include "FoundationDitchGenerator.h"
#include "SliceOperator.h"
#include "Cutter.h"
#include "GeneratorHelp.h"

#include <osg/PolygonMode>
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
		TrangleVisitor() {}

		inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3) /*const*/
		{
			if (!_va) return;

			const osg::Vec3d& v1 = _va->at(p1);
			const osg::Vec3d& v2 = _va->at(p2);
			const osg::Vec3d& v3 = _va->at(p3);

			Point_2 point1(v1.x(), v1.y());
			Point_2 point2(v2.x(), v2.y());
			Point_2 point3(v3.x(), v3.y());
			if (point1 == point2 || point2 == point3 || point3 == point1)
			{
				return;
			}

			Polygon_2 triPolygon;
			triPolygon.push_back(point1);
			triPolygon.push_back(point2);
			triPolygon.push_back(point3);
			if (triPolygon.is_clockwise_oriented())
			{
				triPolygon.reverse_orientation();
			}

			if (_clipPolygon.bbox().xmax() < triPolygon.bbox().xmin() ||
				_clipPolygon.bbox().xmin() > triPolygon.bbox().xmax() ||
				_clipPolygon.bbox().ymax() < triPolygon.bbox().ymin() ||
				_clipPolygon.bbox().ymin() > triPolygon.bbox().ymax())
			{
				return;
			}

			Pwh_list_2 intR;
			CGAL::intersection(_clipPolygon, triPolygon, std::back_inserter(intR));

			Pwh_list_2::const_iterator it;
			for (it = intR.begin(); it != intR.end(); ++it)
			{
				_resultPolygons.push_back(*it);
			}
		}

		void setClipPolygon(const Polygon_2& polygon)
		{
			_clipPolygon = polygon;
		}

		void setGeom(osg::Geometry* geom)
		{
			_va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		}

		void setColor(const osg::Vec4& color)
		{
			_color = color;
		}

		void join()
		{
			if (_resultPolygons.empty())
			{
				return;
			}

			osg::Geometry* geom = 0L;
			osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
			for (Pwh_vector_2::const_iterator it = _resultPolygons.begin(); it != _resultPolygons.end(); ++it)
			{
				if ((*it).outer_boundary().is_convex())
				{
					osg::ref_ptr<osg::DrawElementsUInt> polyele = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
					Polygon_2::Vertex_const_iterator pit;
					for (pit = (*it).outer_boundary().vertices_begin();
						pit != (*it).outer_boundary().vertices_end(); ++pit)
					{
						polyele->push_back(va->size());
						va->push_back(osg::Vec3d(CGAL::to_double(pit->x()), CGAL::to_double(pit->y()), 0.0));
					}
					if (!geom) geom = new osg::Geometry;
					geom->addPrimitiveSet(polyele);
				}
				else
				{
					osg::ref_ptr<osg::Vec3Array> noconva = new osg::Vec3Array;
					Polygon_2::Vertex_const_iterator pit;
					for (pit = (*it).outer_boundary().vertices_begin();
						pit != (*it).outer_boundary().vertices_end(); ++pit)
					{
						noconva->push_back(osg::Vec3d(CGAL::to_double(pit->x()), CGAL::to_double(pit->y()), 0.0));
					}

					osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
					ca->push_back(_color);

					osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array;
					na->push_back(osg::Vec3(0, 0, 1));

					osg::ref_ptr<osg::Geometry> noconGeom = new osg::Geometry;
					noconGeom->setVertexArray(noconva);
					noconGeom->setColorArray(ca);
					noconGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
					noconGeom->setNormalArray(na);
					noconGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
					noconGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, noconva->size()));

					osgUtil::Tessellator tscx;
					tscx.setTessellationNormal(osg::Vec3d(0, 0, 1));
					tscx.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
					tscx.setBoundaryOnly(false);
					tscx.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
					tscx.retessellatePolygons(*noconGeom);

					_noConvexGeoms.push_back(noconGeom);
				}
			}

			osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
			ca->push_back(_color);

			osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array;
			na->push_back(osg::Vec3(0, 0, 1));

			geom->setVertexArray(va);
			geom->setColorArray(ca);
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			geom->setNormalArray(na);
			geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

			_geom = geom;
		}

		const std::vector<osg::ref_ptr<osg::Geometry> >& getNoConvGeoms() const
		{
			return _noConvexGeoms;
		}

		osg::Geometry* getResult() const { return _geom.get(); }

		void reset()
		{
			_noConvexGeoms.clear();
			_resultPolygons.clear();
			if (_va) _va->clear();
			_clipPolygon.clear();
		}

	private:
		Polygon_2 _clipPolygon;
		osg::Vec3Array* _va;
		Pwh_vector_2 _resultPolygons;
		osg::Vec4 _color;
		std::vector<osg::ref_ptr<osg::Geometry> > _noConvexGeoms;

		osg::ref_ptr<osg::Geometry> _geom;
	};

	//template<typename ARRAY_TYPE>
	//static void removeRepeat(ARRAY_TYPE* va, double epsilon = FLT_EPSILON)
	//{
	//	ARRAY_TYPE::iterator it, it1;
	//	for (it = ++va->begin(); it != va->end();)
	//	{
	//		bool isRepeat = false;
	//		for (it1 = va->begin(); it1 != it; ++it1)
	//		{
	//			if (osg::equivalent((*it)[0], (*it1)[0], epsilon) ||
	//				osg::equivalent((*it)[1], (*it1)[1], epsilon))
	//			{
	//				isRepeat = true;
	//				break;
	//			}
	//		}
	//		if (isRepeat) it = va->erase(it);
	//		else it++;
	//	}
	//}

	template<typename ARRAY_TYPE = osg::Vec3dArray>
	static void removeRepeat11(ARRAY_TYPE* va, double epsilon = GEO_EPSILOND)
	{
		ARRAY_TYPE::iterator it, it1;
		for (it = va->begin() + 1; it != va->end();)
		{
			it1 = it - 1;
			if (osg::equivalent((double)(*it)[0], (double)(*it1)[0], epsilon) &&
				osg::equivalent((double)(*it)[1], (double)(*it1)[1], epsilon))
			{
				it = va->erase(it);
			}
			else it++;
		}
	}

	struct PolygonCliper :public osg::NodeVisitor
	{
		PolygonCliper(const osg::Vec3d& normal, osg::Vec3Array* va)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _normal(normal), _clipVa(va)
		{
			Polygon_2 clipPolygon;
			for (osg::Vec3Array::iterator it = _clipVa->begin(); it != _clipVa->end(); ++it)
			{
				osg::Vec3d v = (*it);
				clipPolygon.push_back(Point_2(v.x(), v.y()));
			}
			if (clipPolygon.is_clockwise_oriented())
			{
				std::reverse(_clipVa->begin(), _clipVa->end());
			}
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

			std::vector<osg::ref_ptr<osg::Geometry> > geomVec;
			for (unsigned int i = 0; i < polygonGeode.getNumDrawables(); ++i)
			{
				geomVec.push_back(polygonGeode.getDrawable(i)->asGeometry());
			}

			for (unsigned int i = 0; i < geomVec.size(); ++i)
			{
				osg::Geometry* geom = geomVec[i];
				if (!geom) continue;

				osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
				if (!va) continue;

				osg::ref_ptr<osg::Vec4Array> ca = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
				osg::Vec4 color = (ca && !ca->empty()) ? ca->at(0) : osg::Vec4(1, 1, 1, 1);

				osg::TriangleIndexFunctor<TrangleVisitor> triFunctor;
				triFunctor.setClipPolygon(clipPolygon);
				triFunctor.setColor(color);
				triFunctor.setGeom(geom);
				geom->accept(triFunctor);
				triFunctor.join();

				osg::Geometry* gg = triFunctor.getResult();
				if (gg && gg->getNumPrimitiveSets() > 0)
				{
					polygonGeode.replaceDrawable(geom, gg);
				}
				else
				{
					polygonGeode.removeDrawable(geom);
				}

				const std::vector<osg::ref_ptr<osg::Geometry> >& geoms = triFunctor.getNoConvGeoms();
				if (!geoms.empty())
				{
					for (unsigned int i = 0; i < geoms.size(); ++i)
					{
						polygonGeode.addDrawable(geoms[i]);
					}
				}
			}
			geomVec.clear();
		}

	private:
		osg::Vec3d _normal;
		osg::Vec3Array* _clipVa;

		//bool _isCanClip;
	};
}


//void setMaskState(osg::Node* node)
//{
//	osg::Stencil* stencil = new osg::Stencil;
//	stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0u);
//	stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);
//
//	node->getOrCreateStateSet()->setAttributeAndModes(stencil, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
//	//node->getOrCreateStateSet()->setAttribute(new osg::ColorMask(false, false, false, false), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
//	node->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
//}

osg::Node * FoundationDitchGenerator::operator()(osg::Node * geologicalGroup, const osg::Vec3Array * vva)
{
	if (!geologicalGroup) return 0L;
	if (vva->size() < 3) return 0;

	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
	va->reserve(vva->size() + 1);
	for (osg::Vec3Array::const_iterator it = vva->begin(); it != vva->end(); ++it)
	{
		va->push_back(*it);
	}
	removeRepeat11<osg::Vec3Array>(va);
	if (va->size() < 3) return 0L;
	va->push_back(vva->at(0));

	SliceOperator slice(_isOnEarth);
	osg::ref_ptr<osg::Group> sideGroup = new osg::Group;
	//setMaskState(sideGroup);
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
		va->erase(va->end() - 1);
		PolygonCliper pc(osg::Vec3d(0, 0, 1), va);
		bottomNode->accept(pc);
		profileGroup->addChild(bottomNode);
	}

	setMaskState(profileGroup);

	{//test
		osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
		pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
		profileGroup->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		profileGroup->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	}

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