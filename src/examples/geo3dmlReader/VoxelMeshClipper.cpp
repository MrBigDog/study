#include "VoxelMeshClipper.h"
#include "Vector.h"
#include <stdio.h>
#include <string>
#include <fstream>
#include <osg/Math>
#include <osg/TriangleIndexFunctor>
#include <osg/Geode>

namespace gwUtil
{
	namespace
	{
		struct VC_FVF//vertex color
		{
			VC_FVF(const osg::Vec3&v, const osg::Vec4&c)
				:_v(v), _c(c) {}
			inline bool operator == (const VC_FVF& vnc) const
			{
				return (_v == vnc._v) && (_c == vnc._c);
			}
			inline bool operator < (const VC_FVF& vnc) const
			{
				return (_v < vnc._v);
			}
			osg::Vec3 _v;
			osg::Vec4 _c;
		};

		//vertex normal color
		struct VNC_FVF
		{
			VNC_FVF(const osg::Vec3&v, const osg::Vec3&n, const osg::Vec4&c)
				:_v(v), _n(n), _c(c) {}
			inline bool operator == (const VNC_FVF& vnc) const
			{
				return (_v == vnc._v) && (_n == vnc._n) && (_c == vnc._c);
			}
			inline bool operator < (const VNC_FVF& vnc) const
			{
				return (_v < vnc._v);
			}
			osg::Vec3 _v;
			osg::Vec3 _n;
			osg::Vec4 _c;
		};

		//vertex normal texcoord
		struct VNT_FVF
		{
			VNT_FVF(const osg::Vec3&v, const osg::Vec3&n, const osg::Vec2&tc)
				:_v(v), _n(n), _tc(tc) {}
			inline bool operator == (const VNT_FVF& vnt) const
			{
				return (_v == vnt._v) && (_n == vnt._n) && (_tc == vnt._tc);
			}
			inline bool operator < (const VNT_FVF& vnt) const
			{
				return (_v < vnt._v);
			}
			osg::Vec3 _v;
			osg::Vec3 _n;
			osg::Vec2 _tc;
		};

		//vertex normal color texcoord
		struct VNCT_FVF
		{
			VNCT_FVF(const osg::Vec3&v, const osg::Vec3&n, const osg::Vec2&tc)
				:_v(v), _n(n), _tc(tc) {}
			inline bool operator == (const VNCT_FVF& vnct) const
			{
				return (_v == vnct._v) && (_n == vnct._n) && (_c == vnct._c) && (_tc == vnct._tc);
			}
			inline bool operator < (const VNCT_FVF& vnct) const
			{
				return (_v < vnct._v);
			}
			osg::Vec3 _v;
			osg::Vec3 _n;
			osg::Vec4 _c;
			osg::Vec2 _tc;
		};

		struct TrangleTraverser
		{
			osg::ref_ptr<osg::UIntArray> _ia;
			TrangleTraverser() :_ia(new osg::UIntArray) {}
			void reset() { _ia->clear(); }
			inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3)
			{
				_ia->push_back(p1);
				_ia->push_back(p2);
				_ia->push_back(p3);
			}
		};

		template<typename T>
		void lerp(const T& from, const T& to, float t, T& result)
		{
			result = from + (to - from)*t;
		}

		template<typename T>
		void getIntersect(const T& a, const T& b, const T& c, float coeff1, float coeff2, T& intersect1, T& intersect2)
		{
			lerp(a, b, coeff1, intersect1);
			lerp(a, c, coeff2, intersect2);
		}

		void getlinePlaneCoefficient(
			const osg::Vec3d& a,
			const osg::Vec3d& b,
			const osg::Vec3d& c,
			const osg::Vec3d& planeNormal,
			const osg::Vec3d& planePoint,
			float& intersect1coeff,
			float& intersect2coeff)
		{
			osg::Vec3 line1; sub3(b, a, line1);
			osg::Vec3 line2; sub3(c, a, line2);
			intersect1coeff = linePlaneCoefficient(a, line1, planeNormal, planePoint);
			intersect2coeff = linePlaneCoefficient(a, line2, planeNormal, planePoint);
		}

		//-------------------------------------------------------------------------------------------------------------
		template<typename T>
		struct RefPolyline : public osg::Referenced
		{
			typedef std::vector</*osg::Vec4d*/T> Polyline;
			Polyline _polyline;
			void reverse()
			{
				unsigned int s = 0;
				unsigned int e = _polyline.size() - 1;
				for (; s < e; ++s, --e)
					std::swap(_polyline[s], _polyline[e]);
			}
		};

		template<typename T>
		class PolylineConnector//most from osg PlaneIntersector.cpp
		{
		public:
			typedef typename std::map< T, osg::ref_ptr<RefPolyline<T> > > PolylineMap;
			typedef typename std::map< T, osg::ref_ptr<RefPolyline<T> > >::iterator PolylineMapItr;

			typedef std::vector< osg::ref_ptr<RefPolyline<T> > > PolylineList;
			PolylineList    _polylines;
			PolylineMap     _startPolylineMap;
			PolylineMap     _endPolylineMap;

			void add(const T& v1, const T& v2)
			{
				if (v1 == v2) return;
				PolylineMapItr v1_start_itr = _startPolylineMap.find(v1);
				PolylineMapItr v1_end_itr = _endPolylineMap.find(v1);
				PolylineMapItr v2_start_itr = _startPolylineMap.find(v2);
				PolylineMapItr v2_end_itr = _endPolylineMap.find(v2);
				unsigned int v1_connections = 0;
				if (v1_start_itr != _startPolylineMap.end()) ++v1_connections;
				if (v1_end_itr != _endPolylineMap.end()) ++v1_connections;
				unsigned int v2_connections = 0;
				if (v2_start_itr != _startPolylineMap.end()) ++v2_connections;
				if (v2_end_itr != _endPolylineMap.end()) ++v2_connections;

				if (v1_connections == 0) // v1 is no connected to anything.
				{
					if (v2_connections == 0)
					{
						newline(v1, v2);// new polyline
					}
					else if (v2_connections == 1)
					{
						if (v2_start_itr != _startPolylineMap.end())
							insertAtStart(v1, v2_start_itr);
						else if (v2_end_itr != _endPolylineMap.end())
							insertAtEnd(v1, v2_end_itr);
						else OSG_NOTICE << "Error: should not get here!" << std::endl;
					}
					else {}
				}
				else if (v2_connections == 0) // v1 is no connected to anything.
				{
					if (v1_connections == 1)
					{
						if (v1_start_itr != _startPolylineMap.end())
							insertAtStart(v2, v1_start_itr);
						else if (v1_end_itr != _endPolylineMap.end())
							insertAtEnd(v2, v1_end_itr);
						else OSG_NOTICE << "Error: should not get here!" << std::endl;
					}
					else {}
				}
				else
				{
					bool v1_connected_to_start = v1_start_itr != _startPolylineMap.end();
					bool v1_connected_to_end = v1_end_itr != _endPolylineMap.end();
					bool v2_connected_to_start = v2_start_itr != _startPolylineMap.end();
					bool v2_connected_to_end = v2_end_itr != _endPolylineMap.end();
					if (v1_connected_to_start)
					{
						if (v2_connected_to_start)
							fuse_start_to_start(v1_start_itr, v2_start_itr);
						else if (v2_connected_to_end)
							fuse_start_to_end(v1_start_itr, v2_end_itr);
						else OSG_NOTICE << "Error: should not get here!" << std::endl;
					}
					else if (v1_connected_to_end)
					{
						if (v2_connected_to_start)
							fuse_start_to_end(v2_start_itr, v1_end_itr);
						else if (v2_connected_to_end)
							fuse_end_to_end(v1_end_itr, v2_end_itr);
						else OSG_NOTICE << "Error: should not get here!" << std::endl;
					}
					else OSG_NOTICE << "Error: should not get here!" << std::endl;
				}
			}

			void newline(const T& v1, const T& v2)
			{
				RefPolyline<T>* polyline = new RefPolyline<T>;
				polyline->_polyline.push_back(v1);
				polyline->_polyline.push_back(v2);
				_startPolylineMap[v1] = polyline;
				_endPolylineMap[v2] = polyline;
			}

			void insertAtStart(const T& v, PolylineMapItr v_start_itr)
			{
				RefPolyline<T>* polyline = v_start_itr->second.get();
				polyline->_polyline.insert(polyline->_polyline.begin(), v);
				_startPolylineMap[v] = polyline;
				_startPolylineMap.erase(v_start_itr);
			}

			void insertAtEnd(const T& v, PolylineMapItr v_end_itr)
			{
				RefPolyline<T>* polyline = v_end_itr->second.get();
				polyline->_polyline.push_back(v);
				_endPolylineMap[v] = polyline;
				_endPolylineMap.erase(v_end_itr);
			}

			void fuse_start_to_start(PolylineMapItr start1_itr, PolylineMapItr start2_itr)
			{
				osg::ref_ptr<RefPolyline<T> > poly1 = start1_itr->second;
				osg::ref_ptr<RefPolyline<T> > poly2 = start2_itr->second;
				PolylineMapItr end1_itr = _endPolylineMap.find(poly1->_polyline.back());
				PolylineMapItr end2_itr = _endPolylineMap.find(poly2->_polyline.back());
				_startPolylineMap.erase(start1_itr);
				_startPolylineMap.erase(start2_itr);
				_endPolylineMap.erase(end1_itr);
				_endPolylineMap.erase(end2_itr);
				poly1->reverse();
				poly1->_polyline.insert(poly1->_polyline.end(),
					poly2->_polyline.begin(), poly2->_polyline.end());
				_startPolylineMap[poly1->_polyline.front()] = poly1;
				_endPolylineMap[poly1->_polyline.back()] = poly1;
			}

			void fuse_start_to_end(PolylineMapItr start_itr, PolylineMapItr end_itr)
			{
				osg::ref_ptr<RefPolyline<T> > end_poly = end_itr->second;
				osg::ref_ptr<RefPolyline<T> > start_poly = start_itr->second;
				PolylineMapItr end_start_poly_itr = _endPolylineMap.find(start_poly->_polyline.back());
				end_poly->_polyline.insert(end_poly->_polyline.end(),
					start_poly->_polyline.begin(), start_poly->_polyline.end());
				end_start_poly_itr->second = end_poly;
				_endPolylineMap.erase(end_itr);
				_startPolylineMap.erase(start_itr);
				if (end_poly == start_poly)
				{
					_polylines.push_back(end_poly);
				}
			}

			void fuse_end_to_end(PolylineMapItr end1_itr, PolylineMapItr end2_itr)
			{
				osg::ref_ptr<RefPolyline<T> > poly1 = end1_itr->second;
				osg::ref_ptr<RefPolyline<T> > poly2 = end2_itr->second;
				PolylineMapItr start1_itr = _startPolylineMap.find(poly1->_polyline.front());
				PolylineMapItr start2_itr = _startPolylineMap.find(poly2->_polyline.front());
				_startPolylineMap.erase(start1_itr);
				_startPolylineMap.erase(start2_itr);
				_endPolylineMap.erase(end1_itr);
				_endPolylineMap.erase(end2_itr);
				poly2->reverse();
				poly1->_polyline.insert(poly1->_polyline.end(),
					poly2->_polyline.begin(), poly2->_polyline.end());
				_startPolylineMap[poly1->_polyline.front()] = poly1;
				_endPolylineMap[poly1->_polyline.back()] = poly1;
			}

			void consolidatePolylineLists()
			{
				for (PolylineMapItr sitr = _startPolylineMap.begin(); sitr != _startPolylineMap.end(); ++sitr)
				{
					_polylines.push_back(sitr->second);
				}
			}
		};

		void intersectTrangle(const osg::Vec3Array* va,
			const osg::Vec4Array* ca,
			int ia, int ib, int ic,
			const osg::Vec3& planeNormal,
			const osg::Vec3& planePoint,
			PolylineConnector<VC_FVF>& connector)
		{
			if (!va || !ca) return;
			if (ia < 0 || ib < 0 || ic < 0) return;

			float coeff1, coeff2;
			getlinePlaneCoefficient((*va)[ia], (*va)[ib], (*va)[ic], planeNormal, planePoint, coeff1, coeff2);

			osg::Vec3 intersect1, intersect2;
			osg::Vec4 color1, color2;

			getIntersect((*va)[ia], (*va)[ib], (*va)[ic], coeff1, coeff2, intersect1, intersect2);
			getIntersect((*ca)[ia], (*ca)[ib], (*ca)[ic], coeff1, coeff2, color1, color2);

			VC_FVF res1(intersect1, color1);
			VC_FVF res2(intersect2, color2);
			connector.add(res1, res2);
		}
	}

	//----------------------------------------------------------------------------------------------------------------------------
	VolelMeshClipper::VolelMeshClipper(FVF_TYPE type) :_fvfType(type) {}
	VolelMeshClipper::~VolelMeshClipper() {}

	osg::Node* VolelMeshClipper::clip(osg::Geometry* geom, const osg::Vec3d& planePoint, const osg::Vec3d& planeNormal)
	{
		if (!geom) return 0L;

		unsigned int indexCount = 0;
		osg::ref_ptr<osg::Vec3Array>va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		osg::ref_ptr<osg::Vec3Array>na = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
		osg::ref_ptr<osg::Vec4Array>ca = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
		osg::ref_ptr<osg::UIntArray> ia = 0L;

		//¶¥µãË÷Òý;
		osg::TriangleIndexFunctor<TrangleTraverser> tt;
		geom->accept(tt);
		if (tt._ia && !tt._ia->empty())
		{
			ia = new osg::UIntArray;
			ia->reserve(tt._ia->size());
			for (osg::UIntArray::const_iterator it = tt._ia->begin(); it != tt._ia->end(); ++it)
				ia->push_back(*it);
			indexCount = ia->size();
			tt.reset();
		}

		PolylineConnector<VC_FVF> connector;

		int faceCount = indexCount / 3;
		osg::Plane plane(planeNormal, planePoint);
		for (int i = 0; i < faceCount; ++i)
		{
			int i1 = (*ia)[i * 3 + 0];
			int i2 = (*ia)[i * 3 + 1];
			int i3 = (*ia)[i * 3 + 2];

			osg::Vec3 v1 = (*va)[i1];
			osg::Vec3 v2 = (*va)[i2];
			osg::Vec3 v3 = (*va)[i3];

			float v1ToPlane = plane.distance(v1);
			float v2ToPlane = plane.distance(v2);
			float v3ToPlane = plane.distance(v3);

			bool v1Left = v1ToPlane < 0;
			bool v2Left = v2ToPlane < 0;
			bool v3Left = v3ToPlane < 0;
			bool v1OnPlane = osg::equivalent(v1ToPlane, 0.0f);
			bool v2OnPlane = osg::equivalent(v2ToPlane, 0.0f);
			bool v3OnPlane = osg::equivalent(v3ToPlane, 0.0f);
			int pointsToLeft = (v1Left ? 1 : 0) + (v2Left ? 1 : 0) + (v3Left ? 1 : 0);
			int pointsOnPlane = (v1OnPlane ? 1 : 0) + (v2OnPlane ? 1 : 0) + (v3OnPlane ? 1 : 0);

			if (pointsOnPlane == 3)
			{
				continue;
			}
			else if (pointsOnPlane == 0)
			{
				if (pointsToLeft == 3 || pointsToLeft == 0)
				{
					continue;
				}
				else if (pointsToLeft == 1)// One vertex to left
				{
					int ia = -1, ib = -1, ic = -1;
					if (v1Left) { ia = i1; ib = i2; ic = i3; }
					else if (v2Left) { ia = i2; ib = i1; ic = i3; }
					else if (v3Left) { ia = i3; ib = i1; ic = i2; }
					intersectTrangle(va, ca, ia, ib, ic, planeNormal, planePoint, connector);
				}
				else if (pointsToLeft == 2)
				{
					int ia = -1, ib = -1, ic = -1;
					if (!v1Left) { ia = i1; ib = i2; ic = i3; }
					else if (!v2Left) { ia = i2; ib = i1; ic = i3; }
					else if (!v3Left) { ia = i3; ib = i1; ic = i2; }
					intersectTrangle(va, ca, ia, ib, ic, planeNormal, planePoint, connector);
				}
			}
			else if (pointsOnPlane == 1)
			{
				if (v1OnPlane)
				{
					if (pointsToLeft == 1)
					{
						int ia = -1, ib = -1, ic = -1;
						if (v2Left) { ia = i2; ib = i1; ic = i3; }
						else if (v3Left) { ia = i3; ib = i1; ic = i2; }
						intersectTrangle(va, ca, ia, ib, ic, planeNormal, planePoint, connector);
					}
				}
				else if (v2OnPlane)
				{
					if (pointsToLeft == 1)
					{
						int ia = -1, ib = -1, ic = -1;
						if (v1Left) { ia = i1; ib = i2; ic = i3; }
						else if (v3Left) { ia = i3; ib = i1; ic = i2; }
						intersectTrangle(va, ca, ia, ib, ic, planeNormal, planePoint, connector);
					}
				}
				else if (v3OnPlane)
				{
					if (pointsToLeft == 1)
					{
						int ia = -1, ib = -1, ic = -1;
						if (v1Left) { ia = i1; ib = i2; ic = i3; }
						else if (v2Left) { ia = i2; ib = i1; ic = i3; }
						intersectTrangle(va, ca, ia, ib, ic, planeNormal, planePoint, connector);
					}
				}
			}
			else if (pointsOnPlane == 2)
			{
				if (!v1OnPlane)
				{
					VC_FVF res1((*va)[i2], (*ca)[i2]);
					VC_FVF res2((*va)[i3], (*ca)[i3]);
					connector.add(res1, res2);
				}
				else if (!v2OnPlane)
				{
					VC_FVF res1((*va)[i1], (*ca)[i1]);
					VC_FVF res2((*va)[i3], (*ca)[i3]);
					connector.add(res1, res2);
				}
				else if (!v3OnPlane)
				{
					VC_FVF res1((*va)[i1], (*ca)[i1]);
					VC_FVF res2((*va)[i2], (*ca)[i2]);
					connector.add(res1, res2);
				}
			}
		}

		osg::ref_ptr<osg::Geode> geode = 0L;
		connector.consolidatePolylineLists();
		for (unsigned int pi = 0; pi < connector._polylines.size(); ++pi)
		{
			if (connector._polylines[pi]->_polyline.size() < 3)
			{
				continue;
			}
			osg::ref_ptr<osg::Vec3Array> pva = new osg::Vec3Array;
			osg::ref_ptr<osg::Vec4Array> pca = new osg::Vec4Array;
			for (unsigned int fi = 0; fi < connector._polylines[pi]->_polyline.size(); ++fi)
			{
				const VC_FVF& vnc = connector._polylines[pi]->_polyline[fi];
				pva->push_back(vnc._v);
				pca->push_back(vnc._c);
			}
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
			geom->setUseDisplayList(false);
			geom->setUseVertexBufferObjects(true);
			geom->setVertexArray(pva);
			geom->setColorArray(pca, osg::Array::BIND_PER_VERTEX);
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, pva->size()));
			if (!geode)geode = new osg::Geode;
			geode->addDrawable(geom);
		}
		return geode.release();
	}
}