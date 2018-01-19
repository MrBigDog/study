#include "PlaneIntersector.h"
#include <osg/Geometry>
#include <osg/Notify>
#include <osg/io_utils>
#include <osg/TriangleFunctor>

#include <assert.h>
#include <sstream>

const double elipse = FLT_EPSILON;

namespace gwUtil
{
	namespace PlaneIntersectorUtils
	{
		bool isVec4dEqual(const osg::Vec4d& v1, const osg::Vec4d& v2, double elips)
		{
			if (!osg::equivalent(v1[0], v2[0], elips)) return false;
			if (!osg::equivalent(v1[1], v2[1], elips)) return false;
			if (!osg::equivalent(v1[2], v2[2], elips)) return false;
			if (!osg::equivalent(v1[3], v2[3], elips)) return false;
			return true;
		}

		bool isPolylineContain(const std::vector<osg::Vec4d> polyine, const osg::Vec4d& v)
		{
			for (std::vector<osg::Vec4d>::const_iterator it = polyine.begin(); it != polyine.end(); ++it)
			{
				if (isVec4dEqual(v, *it, elipse)) return true;
			}
			return false;
		}

		struct RefPolyline : public osg::Referenced
		{
			typedef std::vector<osg::Vec4d> Polyline;
			Polyline _polyline;
			void reverse()
			{
				unsigned int s = 0;
				unsigned int e = _polyline.size() - 1;
				for (; s < e; ++s, --e)
				{
					std::swap(_polyline[s], _polyline[e]);
				}
			}
		};

		class PolylineConnector
		{
		public:
			typedef std::map<osg::Vec4d, osg::ref_ptr<RefPolyline> > PolylineMap;
			typedef std::vector< osg::ref_ptr<RefPolyline> > PolylineList;

			PolylineList    _polylines;
			PolylineMap     _startPolylineMap;
			PolylineMap     _endPolylineMap;

			osg::ref_ptr<osg::EllipsoidModel>   _em;

			//enum IntersectType
			//{
			//	ZERO_POINT_ON_PLANE, ONE_POINT_ON_PLANE, TWO_POINT_ON_PLANE
			//};

			//struct RecordEdge
			//{
			//	RecordEdge(const osg::Vec4d& vs, const osg::Vec4d& ve)
			//		:_vs(vs), _ve(ve)
			//	{}
			//	osg::Vec4d _vs, _ve;
			//};
			//typedef std::vector<RecordEdge> RecordEdgeVector;

			//RecordEdgeVector _edges;

			//bool isContains(const RecordEdge& edge, const RecordEdgeVector& edges)
			//{
			//	for (RecordEdgeVector::const_iterator it = edges.begin(); it != edges.end(); ++it)
			//	{
			//		if (isVec4dEqual(edge._vs, it->_vs, elipse) && isVec4dEqual(edge._ve, it->_ve, elipse) ||
			//			isVec4dEqual(edge._vs, it->_ve, elipse) && isVec4dEqual(edge._ve, it->_vs, elipse))
			//		{
			//			return true;
			//		}
			//	}
			//	return false;
			//}

			void add(const osg::Vec3d& v1, const osg::Vec3d& v2)
			{
				add(osg::Vec4d(v1, 0.0), osg::Vec4d(v2, 0.0));
			}

			void add(const osg::Vec4d& v1, const osg::Vec4d& v2)
			{
				if (v1 == v2)
				{
					//OSG_NOTICE << "same point" << std::endl;//三角形的三边共线;
					return;
				}

				PolylineMap::iterator v1_start_itr = _startPolylineMap.find(v1);
				PolylineMap::iterator v1_end_itr = _endPolylineMap.find(v1);

				PolylineMap::iterator v2_start_itr = _startPolylineMap.find(v2);
				PolylineMap::iterator v2_end_itr = _endPolylineMap.find(v2);

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
						newline(v1, v2); // new polyline
					}
					else if (v2_connections == 1)
					{
						// v2 must connect to either a start or an end.
						if (v2_start_itr != _startPolylineMap.end())
						{
							insertAtStart(v1, v2_start_itr);
						}
						else if (v2_end_itr != _endPolylineMap.end())
						{
							insertAtEnd(v1, v2_end_itr);
						}
						else
						{
							OSG_NOTICE << "Error: should not get here!" << std::endl;
						}
					}
					else if (v2_connections == 2)
					{
						//newline(v1, v2);//防止破坏闭环结构，新建一条线;
						// v2 connects to a start and an end - must have a loop in the list!
						OSG_NOTICE << "v2=" << v2 << " must connect to a start and an end - must have a loop!!!!!." << std::endl;
					}
				}
				else if (v2_connections == 0) // v1 is no connected to anything.
				{
					if (v1_connections == 1)
					{
						// v1 must connect to either a start or an end.
						if (v1_start_itr != _startPolylineMap.end())
						{
							insertAtStart(v2, v1_start_itr);
						}
						else if (v1_end_itr != _endPolylineMap.end())
						{
							insertAtEnd(v2, v1_end_itr);
						}
						else
						{
							OSG_NOTICE << "Error: should not get here!" << std::endl;
						}
					}
					else if (v1_connections == 2)
					{
						//newline(v1, v2);//防止破坏闭环结构，新建一条线;
						// v1 connects to a start and an end - must have a loop in the list!
						OSG_NOTICE << "v1=" << v1 << " must connect to a start and an end - must have a loop!!!!!." << std::endl;
					}
				}
				else
				{
					// v1 and v2 connect to existing lines, now need to fuse them together.
					bool v1_connected_to_start = v1_start_itr != _startPolylineMap.end();
					bool v1_connected_to_end = v1_end_itr != _endPolylineMap.end();
					bool v2_connected_to_start = v2_start_itr != _startPolylineMap.end();
					bool v2_connected_to_end = v2_end_itr != _endPolylineMap.end();

					if (v1_connected_to_start)
					{
						if (v2_connected_to_start)
						{
							//if (!isSubsetOf(*(v1_start_itr->second), *(v2_start_itr->second)) &&
							//	!isSubsetOf(*(v2_start_itr->second), *(v1_start_itr->second)))
							//{
							fuse_start_to_start(v1_start_itr, v2_start_itr);
							//}
						}
						else if (v2_connected_to_end)
						{
							//if (!isSubsetOf(*(v1_start_itr->second), *(v2_end_itr->second)) &&
							//	!isSubsetOf(*(v2_end_itr->second), *(v1_start_itr->second)))
							{
								if (v1_start_itr->second == v2_end_itr->second)
								{
									//if (v1_start_itr->second->_polyline.size() > 2)//
									{
										osg::ref_ptr<RefPolyline> polyline = v1_start_itr->second.get();
										polyline->_polyline.insert(polyline->_polyline.begin(), v2);
										_endPolylineMap.erase(v2_end_itr);
										_startPolylineMap.erase(v1_start_itr);
										_polylines.push_back(polyline);
									}
								}
								else
								{
									fuse_start_to_end(v1_start_itr, v2_end_itr);
								}
							}
						}
						else
						{
							OSG_NOTICE << "Error: should not get here!" << std::endl;
						}
					}
					else if (v1_connected_to_end)
					{
						if (v2_connected_to_start)
						{
							//if (!isSubsetOf(*(v1_end_itr->second), *(v2_start_itr->second)) &&
							//	!isSubsetOf(*(v2_start_itr->second), *(v1_end_itr->second)))
							{
								if (v1_end_itr->second == v2_start_itr->second)
								{
									//if (v1_end_itr->second->_polyline.size() > 2)
									{
										//insertAtEnd(v2, v1_end_itr);
										osg::ref_ptr<RefPolyline> polyline = v1_end_itr->second.get();
										polyline->_polyline.push_back(v2);
										_endPolylineMap.erase(v1_end_itr);
										_startPolylineMap.erase(v2_start_itr);
										_polylines.push_back(polyline);
									}
								}
								else
								{
									fuse_start_to_end(v2_start_itr, v1_end_itr);
								}
							}
						}
						else if (v2_connected_to_end)
						{
							//if (!isSubsetOf(*(v1_end_itr->second), *(v2_end_itr->second)) &&
							//	!isSubsetOf(*(v2_end_itr->second), *(v1_end_itr->second)))
							{
								fuse_end_to_end(v1_end_itr, v2_end_itr);
							}
						}
						else
						{
							OSG_NOTICE << "Error: should not get here!" << std::endl;
						}
					}
					else
					{
						OSG_NOTICE << "Error: should not get here!" << std::endl;
					}
				}
			}

			//bool isPointInPolyLine(const osg::Vec4d& v)
			//{
			//	for (PolylineList::const_iterator it = _polylines.begin(); it != _polylines.end(); ++it)
			//	{
			//		for (RefPolyline::Polyline::const_iterator vit = (*it)->_polyline.begin(); vit != (*it)->_polyline.end(); ++vit)
			//		{
			//			if (isVec4dEqual(v, *vit, elipse))
			//			{
			//				return true;
			//			}
			//		}
			//	}
			//	for (PolylineMap::const_iterator it = _startPolylineMap.begin(); it != _startPolylineMap.end(); ++it)
			//	{
			//		for (RefPolyline::Polyline::const_iterator vit = (it->second)->_polyline.begin(); vit != (it->second)->_polyline.end(); ++vit)
			//		{
			//			if (isVec4dEqual(v, *vit, elipse))
			//			{
			//				return true;
			//			}
			//		}
			//	}
			//	return false;
			//}

			void newline(const osg::Vec4d& v1, const osg::Vec4d& v2)
			{
				//if (isPointInPolyLine(v1) && isPointInPolyLine(v2))
				//{
				//	//return;
				//}

				RefPolyline* polyline = new RefPolyline;
				polyline->_polyline.push_back(v1);
				polyline->_polyline.push_back(v2);
				_startPolylineMap[v1] = polyline;
				_endPolylineMap[v2] = polyline;
			}

			bool isContains(const osg::Vec4d& v, const RefPolyline::Polyline& pl, unsigned int& ind)
			{
				for (RefPolyline::Polyline::const_iterator it = pl.begin(); it != pl.end(); ++it)
				{
					if (isVec4dEqual(v, *it, elipse))
					{
						ind = it - pl.begin();
						return true;
					}
				}
				return false;
			}

			void insertAtStart(const osg::Vec4d& v, PolylineMap::iterator v_start_itr)
			{
				RefPolyline* polyline = v_start_itr->second.get();

				//unsigned int ind;
				//const RefPolyline::Polyline& pl = polyline->_polyline;
				//if (isContains(v, pl, ind))//分离出闭合区域;
				//{
				//	osg::ref_ptr<RefPolyline> closedPL = new RefPolyline;
				//	closedPL->_polyline.push_back(v);
				//	for (unsigned int i = 0; i <= ind; ++i)
				//	{
				//		closedPL->_polyline.push_back(pl[i]);
				//	}
				//	_polylines.push_back(closedPL);
				//	//删除原来的;
				//	if (ind == polyline->_polyline.size() - 1)
				//	{
				//		OSG_NOTICE << "wrong" << std::endl;
				//	}

				//	for (unsigned int i = 0; i < ind; ++i)
				//	{
				//		polyline->_polyline.erase(polyline->_polyline.begin());
				//	}


				//	if (!isVec4dEqual(v, polyline->_polyline.front(), elipse))
				//	{
				//		OSG_NOTICE << "error" << std::endl;
				//	}

				//	if (polyline->_polyline.size() < 2)
				//	{
				//		PolylineMap::iterator end_itr = _endPolylineMap.find(polyline->_polyline.back());
				//		_endPolylineMap.erase(end_itr);
				//	}
				//	else
				//	{
				//		_startPolylineMap[v] = polyline;
				//	}
				//}
				//else
				{
					// put v1 at the start of its poyline
					polyline->_polyline.insert(polyline->_polyline.begin(), v);
					// reinsert the polyine at the new v1 end
					_startPolylineMap[v] = polyline;
					// remove the original entry
				}
				_startPolylineMap.erase(v_start_itr);
			}

			void insertAtEnd(const osg::Vec4d& v, PolylineMap::iterator v_end_itr)
			{
				// put v1 at the end of its poyline
				RefPolyline* polyline = v_end_itr->second.get();
				//unsigned int ind;
				//const RefPolyline::Polyline& pl = polyline->_polyline;
				//if (isContains(v, pl, ind))//分离出闭合区域;
				//{
				//	if (ind == 0)
				//	{
				//		OSG_NOTICE << "wow" << std::endl;
				//	}
				//	osg::ref_ptr<RefPolyline> closedPL = new RefPolyline;
				//	for (unsigned int i = ind; i < pl.size(); ++i)
				//	{
				//		closedPL->_polyline.push_back(pl[i]);
				//	}
				//	closedPL->_polyline.push_back(v);
				//	_polylines.push_back(closedPL);
				//	//删除原来的;
				//	unsigned int delnum = pl.size() - ind - 1;
				//	for (unsigned int i = 0; i < delnum; ++i)
				//	{
				//		polyline->_polyline.erase(polyline->_polyline.end() - 1);
				//	}

				//	if (!isVec4dEqual(v, polyline->_polyline.back(), elipse))
				//	{
				//		OSG_NOTICE << "error" << std::endl;
				//	}

				//	if (polyline->_polyline.size() < 2)
				//	{
				//		PolylineMap::iterator startpol_itr = _startPolylineMap.find(polyline->_polyline.front());
				//		_startPolylineMap.erase(startpol_itr);
				//	}
				//	else
				//	{
				//		_endPolylineMap[v] = polyline;
				//	}
				//}
				//else
				{
					polyline->_polyline.push_back(v);
					// reinsert the polyine at the new v1 end
					_endPolylineMap[v] = polyline;
				}
				// remove the original entry
				_endPolylineMap.erase(v_end_itr);
			}

			void fuse_start_to_start(PolylineMap::iterator start1_itr, PolylineMap::iterator start2_itr)
			{
				osg::ref_ptr<RefPolyline> poly1 = start1_itr->second;
				osg::ref_ptr<RefPolyline> poly2 = start2_itr->second;

				PolylineMap::iterator end1_itr = _endPolylineMap.find(poly1->_polyline.back());
				PolylineMap::iterator end2_itr = _endPolylineMap.find(poly2->_polyline.back());

				// clean up the iterators associated with the original polylines
				_startPolylineMap.erase(start1_itr);
				_startPolylineMap.erase(start2_itr);
				_endPolylineMap.erase(end1_itr);
				_endPolylineMap.erase(end2_itr);

				// reverse the first polyline
				poly1->reverse();

				// add the second polyline to the first
				poly1->_polyline.insert(poly1->_polyline.end(),
					poly2->_polyline.begin(), poly2->_polyline.end());

				_startPolylineMap[poly1->_polyline.front()] = poly1;
				_endPolylineMap[poly1->_polyline.back()] = poly1;
			}

			void fuse_start_to_end(PolylineMap::iterator start_itr, PolylineMap::iterator end_itr)
			{
				osg::ref_ptr<RefPolyline> end_poly = end_itr->second;
				osg::ref_ptr<RefPolyline> start_poly = start_itr->second;

				PolylineMap::iterator end_start_poly_itr = _endPolylineMap.find(start_poly->_polyline.back());

				// add start_poly to end of end_poly
				end_poly->_polyline.insert(end_poly->_polyline.end(),
					start_poly->_polyline.begin(), start_poly->_polyline.end());

				// reassign the end of the start poly so that it now points to the merged end_poly
				end_start_poly_itr->second = end_poly;


				// remove entries for the end of the end_poly and the start of the start_poly
				_endPolylineMap.erase(end_itr);
				_startPolylineMap.erase(start_itr);

				if (end_poly == start_poly)
				{
					_polylines.push_back(end_poly);
				}
			}

			void fuse_end_to_end(PolylineMap::iterator end1_itr, PolylineMap::iterator end2_itr)
			{
				osg::ref_ptr<RefPolyline> poly1 = end1_itr->second;
				osg::ref_ptr<RefPolyline> poly2 = end2_itr->second;

				PolylineMap::iterator start1_itr = _startPolylineMap.find(poly1->_polyline.front());
				PolylineMap::iterator start2_itr = _startPolylineMap.find(poly2->_polyline.front());

				// clean up the iterators associated with the original polylines
				_startPolylineMap.erase(start1_itr);
				_startPolylineMap.erase(start2_itr);
				_endPolylineMap.erase(end1_itr);
				_endPolylineMap.erase(end2_itr);

				// reverse the first polyline
				poly2->reverse();

				// add the second polyline to the first
				poly1->_polyline.insert(poly1->_polyline.end(),
					poly2->_polyline.begin(), poly2->_polyline.end());

				_startPolylineMap[poly1->_polyline.front()] = poly1;
				_endPolylineMap[poly1->_polyline.back()] = poly1;
			}

			//bool isSubsetOf(const RefPolyline& pl1, const RefPolyline& pl2)
			//{
			//	if (pl1._polyline.empty() || pl2._polyline.empty()) return false;
			//	if (pl1._polyline.size() > pl2._polyline.size()) return false;

			//	RefPolyline::Polyline::const_iterator sit1 = pl1._polyline.begin();
			//	RefPolyline::Polyline::const_iterator sit2 =
			//		std::find(pl2._polyline.begin(), pl2._polyline.end(), *sit1);
			//	if (sit2 == pl2._polyline.end())
			//	{
			//		return false;
			//	}

			//	if (pl2._polyline.end() - sit2 < pl1._polyline.size() &&
			//		sit2 - pl2._polyline.begin() < pl1._polyline.size())
			//	{
			//		return false;
			//	}

			//	if (pl2._polyline.end() - sit2 > pl1._polyline.size())
			//	{
			//		bool issub1 = true;
			//		RefPolyline::Polyline::const_iterator it1, it2;
			//		for (it1 = sit1, it2 = sit2; it1 != pl1._polyline.end(); ++it1, ++it2)
			//		{
			//			if (!isVec4dEqual((*it1), *it2, elipse))
			//			{
			//				issub1 = false;
			//				break;
			//			}
			//		}
			//		if (issub1)
			//		{
			//			return true;
			//		}
			//	}
			//	if (sit2 - pl2._polyline.begin() > pl1._polyline.size())
			//	{
			//		bool issub2 = true;
			//		RefPolyline::Polyline::const_iterator it1, it2;
			//		for (it1 = sit1, it2 = sit2; it1 != pl1._polyline.end(); ++it1, --it2)
			//		{
			//			if (!isVec4dEqual((*it1), *it2, elipse))
			//			{
			//				issub2 = false;
			//				break;
			//			}
			//		}
			//		if (issub2)
			//		{
			//			return true;
			//		}
			//	}
			//	return false;
			//}


			void consolidatePolylineLists()
			{
				// move the remaining open ended line segments into the polyline list
				for (PolylineMap::iterator sitr = _startPolylineMap.begin();
					sitr != _startPolylineMap.end();
					++sitr)
				{
					_polylines.push_back(sitr->second);
				}

				//if (_polylines.empty())
				//{
				//	return;
				//}
				////删除重复的线段;
				//for (PolylineConnector::PolylineList::iterator it = _polylines.begin(); it != _polylines.end() - 1;)
				//{
				//	bool isrepeat = false;
				//	for (PolylineConnector::PolylineList::iterator it1 = _polylines.begin(); it1 != _polylines.end(); ++it1)
				//	{
				//		if ((*it) == (*it1))
				//		{
				//			continue;
				//		}
				//		if (isSubsetOf(**it, **it1))
				//		{
				//			isrepeat = true;
				//			break;
				//		}
				//	}
				//	if (isrepeat)
				//	{
				//		it = _polylines.erase(it);
				//	}
				//	else
				//	{
				//		++it;
				//	}
				//}
			}
		};

		struct TriangleIntersector
		{
			struct IntersectedEdge
			{
				IntersectedEdge(const osg::Vec4d& vs, const osg::Vec4d& ve)
					:_vs(vs), _ve(ve)
				{}
				osg::Vec4d _vs, _ve;
			};
			typedef std::vector<IntersectedEdge> IntersectedEdgeVector;


			bool isContain(const osg::Vec4d& v1, const osg::Vec4d& v2, const IntersectedEdgeVector& intersectedEdges)
			{
				IntersectedEdgeVector::const_iterator it;
				for (it = intersectedEdges.begin(); it != intersectedEdges.end(); ++it)
				{
					if ((isVec4dEqual(it->_vs, v1, FLT_EPSILON) && isVec4dEqual(it->_ve, v2, FLT_EPSILON)) ||
						(isVec4dEqual(it->_vs, v2, FLT_EPSILON) && isVec4dEqual(it->_ve, v1, FLT_EPSILON)))
					{
						return true;
					}
				}
				return false;
			}

			osg::Plane                              _plane;
			osg::Polytope                           _polytope;
			bool                                    _hit;
			osg::ref_ptr<osg::RefMatrix>            _matrix;
			bool                                    _recordHeightsAsAttributes;
			osg::ref_ptr<osg::EllipsoidModel>       _em;
			bool                                    _limitOneIntersection;

			PolylineConnector _polylineConnector;
			PolylineConnector _polylineConnector1;
			IntersectedEdgeVector _intersectedEdges;

			//
			TriangleIntersector() :
				_hit(false),
				_recordHeightsAsAttributes(false),
				_limitOneIntersection(false) {}

			void set(const osg::Plane& plane, const osg::Polytope& polytope, osg::RefMatrix* matrix, bool recordHeightsAsAttributes, osg::EllipsoidModel* em)
			{
				_plane = plane;
				_polytope = polytope;
				_hit = false;
				_matrix = matrix;
				_recordHeightsAsAttributes = recordHeightsAsAttributes;
				_em = em;
			}

			inline double distance(const osg::Plane& plane, const osg::Vec4d& v) const
			{
				return plane[0] * v.x() +
					plane[1] * v.y() +
					plane[2] * v.z() +
					plane[3];
			}

			inline void add(osg::Vec4d& vs, osg::Vec4d& ve, bool isEdgeIntersect)
			{
				if (_polytope.getPlaneList().empty())
				{
					if (isEdgeIntersect)
					{
						_polylineConnector1.add(vs, ve);
					}
					else
					{
						_polylineConnector.add(vs, ve);
					}
				}
				else
				{
					for (osg::Polytope::PlaneList::iterator itr = _polytope.getPlaneList().begin();
						itr != _polytope.getPlaneList().end();
						++itr)
					{
						osg::Plane& plane = *itr;
						double ds = distance(plane, vs);
						double de = distance(plane, ve);

						if (ds < 0.0)
						{
							if (de < 0.0)
							{
								return;
							}
							double div = 1.0 / (de - ds);
							vs = vs*(de*div) - ve*(ds*div);
						}
						else if (de < 0.0)
						{
							double div = 1.0 / (ds - de);
							ve = ve*(ds*div) - vs*(de*div);
						}
					}
					if (isEdgeIntersect)
					{
						_polylineConnector1.add(vs, ve);
					}
					else
					{
						_polylineConnector.add(vs, ve);
					}
				}
			}

			inline void operator () (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool)
			{
				if (_limitOneIntersection && _hit) return;

				double d1 = _plane.distance(v1);
				double d2 = _plane.distance(v2);
				double d3 = _plane.distance(v3);

				unsigned int numBelow = 0;
				unsigned int numAbove = 0;
				unsigned int numOnPlane = 0;
				if (osg::equivalent(d1, 0.0, elipse)) ++numOnPlane;
				else if (d1 < 0) ++numBelow;
				else /*if (d1 > 0)*/ ++numAbove;

				if (osg::equivalent(d2, 0.0, elipse)) ++numOnPlane;
				if (d2 < 0) ++numBelow;
				else /*if (d2 > 0)*/ ++numAbove;

				if (osg::equivalent(d3, 0.0, elipse)) ++numOnPlane;
				if (d3 < 0) ++numBelow;
				else /*if (d3 > 0)*/ ++numAbove;

				// trivially discard triangles that are completely one side of the plane
				if (numAbove == 3 || numBelow == 3) return;

				_hit = true;

				osg::Vec4d v[2];
				unsigned int numIntersects = 0;

				osg::Vec4d p1(v1, v1.z());
				osg::Vec4d p2(v2, v2.z());
				osg::Vec4d p3(v3, v3.z());
				if (_em.valid())
				{
					double latitude, longitude, height;
					if (_matrix.valid())
					{
						osg::Vec3d tp = v1 * (*_matrix);
						_em->convertXYZToLatLongHeight(tp.x(), tp.y(), tp.z(), latitude, longitude, height);
						p1[3] = height;

						tp = v2 * (*_matrix);
						_em->convertXYZToLatLongHeight(tp.x(), tp.y(), tp.z(), latitude, longitude, height);
						p2[3] = height;

						tp = v3 * (*_matrix);
						_em->convertXYZToLatLongHeight(tp.x(), tp.y(), tp.z(), latitude, longitude, height);
						p3[3] = height;

					}
					else
					{
						_em->convertXYZToLatLongHeight(v1.x(), v1.y(), v1.z(), latitude, longitude, height);
						p1[3] = height;

						_em->convertXYZToLatLongHeight(v2.x(), v2.y(), v2.z(), latitude, longitude, height);
						p2[3] = height;

						_em->convertXYZToLatLongHeight(v3.x(), v3.y(), v3.z(), latitude, longitude, height);
						p3[3] = height;
					}
				}

				if (numOnPlane == 0)
				{
					if (d1*d2 < 0.0)
					{
						// edge 12 itersects
						double div = 1.0 / (d2 - d1);
						v[numIntersects++] = p1* (d2*div) - p2 * (d1*div);
					}
					if (d2*d3 < 0.0)
					{
						// edge 23 itersects
						double div = 1.0 / (d3 - d2);
						v[numIntersects++] = p2* (d3*div) - p3 * (d2*div);
					}
					if (d1*d3 < 0.0)
					{
						if (numIntersects < 2)
						{
							// edge 13 itersects
							double div = 1.0 / (d3 - d1);
							v[numIntersects++] = p1* (d3*div) - p3 * (d1*div);
						}
						else
						{
							OSG_NOTICE << "!!! too many intersecting edges found !!!" << std::endl;
						}
					}
					add(v[0], v[1], false);
				}
				else if (numOnPlane == 3)
				{
				}
				else if (numOnPlane == 2)
				{
					if (!osg::equivalent(d1, 0.0, elipse))
					{
						if (!isContain(p2, p3, _intersectedEdges))
						{
							v[numIntersects++] = p2;
							v[numIntersects++] = p3;
							add(v[0], v[1], true);
							_intersectedEdges.push_back(IntersectedEdge(v[0], v[1]));
						}
					}
					else if (!osg::equivalent(d2, 0.0, elipse))
					{
						if (!isContain(p1, p3, _intersectedEdges))
						{
							v[numIntersects++] = p1;
							v[numIntersects++] = p3;
							add(v[0], v[1], true);
							_intersectedEdges.push_back(IntersectedEdge(v[0], v[1]));
						}
					}
					else if (!osg::equivalent(d3, 0.0, elipse))
					{
						if (!isContain(p1, p2, _intersectedEdges))
						{
							v[numIntersects++] = p1;
							v[numIntersects++] = p2;
							add(v[0], v[1], true);
							_intersectedEdges.push_back(IntersectedEdge(v[0], v[1]));
						}
					}
				}
				else if (numOnPlane == 1)
				{
					if (osg::equivalent(d1, 0.0, elipse))
					{
						if (d2*d3 < 0.0)
						{
							v[numIntersects++] = p1;
							double div = 1.0 / (d3 - d2);
							v[numIntersects++] = p2* (d3*div) - p3 * (d2*div);
							add(v[0], v[1], false);
						}
					}
					else if (osg::equivalent(d2, 0.0, elipse))
					{
						if (d1*d3 < 0.0)
						{
							v[numIntersects++] = p2;
							double div = 1.0 / (d3 - d1);
							v[numIntersects++] = p1* (d3*div) - p3 * (d1*div);
							add(v[0], v[1], false);
						}
					}
					else if (osg::equivalent(d3, 0.0, elipse))
					{
						if (d1*d2 < 0.0)
						{
							v[numIntersects++] = p3;
							double div = 1.0 / (d2 - d1);
							v[numIntersects++] = p1* (d2*div) - p2 * (d1*div);
							add(v[0], v[1], false);
						}
					}
				}
			}
		};

		struct Junction :public osg::Referenced
		{
			unsigned int _index;
			typedef std::vector<std::pair<unsigned int, osg::ref_ptr<RefPolyline> > > PolylineVec;
			PolylineVec _polylines;
			Junction(unsigned int ind) :_index(ind) {}
			void addPolyline(unsigned int ind, RefPolyline* polyline)
			{
				_polylines.push_back(std::pair<unsigned int, osg::ref_ptr<RefPolyline> >(ind, polyline));
			}
		};
		typedef std::vector<osg::ref_ptr<Junction> > JunctionVector;

		//
		void mergeLine(PolylineConnector::PolylineList& polylist, const PolylineConnector::PolylineList& additionalPl)
		{
			//@todo自相交的分开;--在上面改 在insertAtStart那里改;
			PolylineConnector::PolylineList::const_iterator ait;
			for (ait = additionalPl.begin(); ait != additionalPl.end(); ++ait)
			{
				if (ait->get()->_polyline.size() < 2) continue;

				JunctionVector junctionVec;

				RefPolyline* apolylien = *ait;
				for (RefPolyline::Polyline::const_iterator vait = apolylien->_polyline.begin(); vait != apolylien->_polyline.end(); ++vait)
				{
					osg::Vec4d v = *vait;
					osg::ref_ptr<Junction> junction = 0L;
					PolylineConnector::PolylineList::const_iterator pit;
					for (pit = polylist.begin(); pit != polylist.end(); ++pit)
					{
						RefPolyline* pl = *pit;
						if (pl->_polyline.size() < 2)
							continue;

						osg::Vec4d s = *(pl->_polyline.begin());
						osg::Vec4d e = *(pl->_polyline.rbegin());
						if (isVec4dEqual(s, e, elipse))//自成一环;
						{
							continue;
						}

						if (isVec4dEqual(v, s, elipse))
						{
							if (!junction)junction = new Junction(vait - apolylien->_polyline.begin());
							junction->addPolyline(0, pl);
						}
						else if (isVec4dEqual(v, e, elipse))
						{
							if (!junction)junction = new Junction(vait - apolylien->_polyline.begin());
							junction->addPolyline(pl->_polyline.size() - 1, pl);
						}
					}
					if (!junction) continue;
					junctionVec.push_back(junction);
				}

				if (junctionVec.empty()) continue;

				unsigned int lastIndex = (*junctionVec.rbegin())->_index;
				for (JunctionVector::const_iterator jit = junctionVec.begin(); jit != junctionVec.end(); ++jit)
				{
					Junction* junction = *jit;
					if (!junction) continue;

					unsigned int currentIndex = junction->_index;
					if (currentIndex >= lastIndex)
					{
						continue;
					}

					Junction::PolylineVec::const_iterator pit;
					for (pit = junction->_polylines.begin(); pit != junction->_polylines.end(); ++pit)
					{
						unsigned int polylineInd = pit->first;
						if (polylineInd == 0)
						{
							for (unsigned int i = currentIndex + 1; i <= lastIndex; ++i)
							{
								pit->second->_polyline.insert(pit->second->_polyline.begin(), apolylien->_polyline.at(i));
							}
						}
						else
						{
							for (unsigned int i = currentIndex + 1; i <= lastIndex; ++i)
							{
								pit->second->_polyline.push_back(apolylien->_polyline.at(i));
							}
						}
					}
				}
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//  PlaneIntersector
	//
	PlaneIntersector::PlaneIntersector(const osg::Plane& plane, const osg::Polytope& boundingPolytope)
		: _parent(0)
		, _recordHeightsAsAttributes(false)
		, _plane(plane)
		, _polytope(boundingPolytope)
	{ }

	PlaneIntersector::PlaneIntersector(CoordinateFrame cf, const osg::Plane& plane, const osg::Polytope& boundingPolytope)
		: Intersector(cf)
		, _parent(0)
		, _recordHeightsAsAttributes(false)
		, _plane(plane)
		, _polytope(boundingPolytope)
	{ }

	osgUtil::Intersector* PlaneIntersector::clone(osgUtil::IntersectionVisitor& iv)
	{
		if (_coordinateFrame == MODEL && iv.getModelMatrix() == 0)
		{
			osg::ref_ptr<PlaneIntersector> pi = new PlaneIntersector(_plane, _polytope);
			pi->_parent = this;
			pi->_intersectionLimit = this->_intersectionLimit;
			pi->_recordHeightsAsAttributes = _recordHeightsAsAttributes;
			pi->_em = _em;
			return pi.release();
		}

		osg::Matrix matrix;
		switch (_coordinateFrame)
		{
		case(WINDOW):
			if (iv.getWindowMatrix()) matrix.preMult(*iv.getWindowMatrix());
			if (iv.getProjectionMatrix()) matrix.preMult(*iv.getProjectionMatrix());
			if (iv.getViewMatrix()) matrix.preMult(*iv.getViewMatrix());
			if (iv.getModelMatrix()) matrix.preMult(*iv.getModelMatrix());
			break;
		case(PROJECTION):
			if (iv.getProjectionMatrix()) matrix.preMult(*iv.getProjectionMatrix());
			if (iv.getViewMatrix()) matrix.preMult(*iv.getViewMatrix());
			if (iv.getModelMatrix()) matrix.preMult(*iv.getModelMatrix());
			break;
		case(VIEW):
			if (iv.getViewMatrix()) matrix.preMult(*iv.getViewMatrix());
			if (iv.getModelMatrix()) matrix.preMult(*iv.getModelMatrix());
			break;
		case(MODEL):
			if (iv.getModelMatrix()) matrix = *iv.getModelMatrix();
			break;
		}

		osg::Plane plane = _plane;
		plane.transformProvidingInverse(matrix);

		osg::Polytope transformedPolytope;
		transformedPolytope.setAndTransformProvidingInverse(_polytope, matrix);

		osg::ref_ptr<PlaneIntersector> pi = new PlaneIntersector(plane, transformedPolytope);
		pi->_parent = this;
		pi->_intersectionLimit = this->_intersectionLimit;
		pi->_recordHeightsAsAttributes = _recordHeightsAsAttributes;
		pi->_em = _em;
		return pi.release();
	}

	bool PlaneIntersector::enter(const osg::Node& node)
	{
		if (reachedLimit()) return false;
		return !node.isCullingActive() ||
			(_plane.intersect(node.getBound()) == 0 && _polytope.contains(node.getBound()));
	}

	void PlaneIntersector::leave()
	{
		// do nothing.
	}

	void PlaneIntersector::intersect(osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable)
	{
		if (reachedLimit()) return;
		if (_plane.intersect(drawable->getBoundingBox()) != 0) return;
		if (!_polytope.contains(drawable->getBoundingBox())) return;

		osg::TriangleFunctor<PlaneIntersectorUtils::TriangleIntersector> ti;
		ti.set(_plane, _polytope, iv.getModelMatrix(), _recordHeightsAsAttributes, _em.get());
		ti._limitOneIntersection = (_intersectionLimit == LIMIT_ONE_PER_DRAWABLE || _intersectionLimit == LIMIT_ONE);
		drawable->accept(ti);

		ti._polylineConnector.consolidatePolylineLists();
		ti._polylineConnector1.consolidatePolylineLists();
		mergeLine(ti._polylineConnector._polylines, ti._polylineConnector1._polylines);

		if (ti._hit)
		{
			Intersections& intersections = getIntersections();
			for (PlaneIntersectorUtils::PolylineConnector::PolylineList::iterator pitr = ti._polylineConnector._polylines.begin();
				pitr != ti._polylineConnector._polylines.end();
				++pitr)
			{
				unsigned int pos = intersections.size();

				intersections.push_back(Intersection());
				Intersection& new_intersection = intersections[pos];

				new_intersection.matrix = iv.getModelMatrix();

				new_intersection.polyline.reserve((*pitr)->_polyline.size());
				if (_recordHeightsAsAttributes) new_intersection.attributes.reserve((*pitr)->_polyline.size());

				for (PlaneIntersectorUtils::RefPolyline::Polyline::iterator vitr = (*pitr)->_polyline.begin();
					vitr != (*pitr)->_polyline.end();
					++vitr)
				{
					const osg::Vec4d& v = *vitr;
					new_intersection.polyline.push_back(osg::Vec3d(v.x(), v.y(), v.z()));
					if (_recordHeightsAsAttributes) new_intersection.attributes.push_back(v.w());
				}

				new_intersection.nodePath = iv.getNodePath();
				new_intersection.drawable = drawable;
			}
		}
	}

	void PlaneIntersector::reset()
	{
		Intersector::reset();
		_intersections.clear();
	}
}