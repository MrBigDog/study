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

#include <assert.h>


#pragma comment(lib, "CGAL-vc140-mt-gd-4.11.lib")
#pragma comment(lib, "CGAL_Core-vc140-mt-gd-4.11.lib")
#pragma comment(lib, "libgmp-10.lib")
#pragma comment(lib, "libmpfr-4.lib")

#define COMPUTEJUNCTION_EPSILON FLT_EPSILON//计算Junction;
#define PATHREPEAT_EPSILON 0.0001//判断BooleanPath是否重合;
#define LARGE_DATA 12742000.0

const bool showSlicePoint = 1;
const bool fillRenderMode = true;
//
//typedef CGAL::Cartesian<double>						         Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel	 Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel>                   Traits_2;
typedef CGAL::Arr_face_extended_dcel<Traits_2, unsigned int> Dcel;
typedef CGAL::Arrangement_2<Traits_2, Dcel>                  Ex_arrangement;
typedef Kernel::Point_2                                      Point_2;
typedef Kernel::Segment_2                                    Segment_2;

template<class Arrangement>
osg::Vec3dArray* getCbbArray(typename Arrangement::Ccb_halfedge_const_circulator circ)
{
	typename Arrangement::Ccb_halfedge_const_circulator  curr = circ;
	typename Arrangement::Halfedge_const_handle          he;

	osg::ref_ptr<osg::Vec3dArray> loopVa = new osg::Vec3dArray;
	double sx = CGAL::to_double(curr->source()->point().x());
	double sy = CGAL::to_double(curr->source()->point().y());
	osg::Vec3d sp = osg::Vec3d(sx, sy, 377);
	loopVa->push_back(sp);
	do
	{
		he = curr;
		double x = CGAL::to_double(he->target()->point().x());
		double y = CGAL::to_double(he->target()->point().y());
		osg::Vec3d p = osg::Vec3d(x, y, 377);
		loopVa->push_back(p);
		++curr;
	} while (curr != circ);

	return loopVa.release();
}

typedef std::vector<osg::ref_ptr<osg::Vec3dArray> > Vec3dArrays;
struct DLoop : public osg::Referenced
{
	DLoop() {}
	osg::ref_ptr<osg::Vec3dArray> _va;
	Vec3dArrays _holes;
};
typedef std::vector<osg::ref_ptr<DLoop> > DLoops;


namespace
{
	//	// 参考《 Real-Time Collision Detection 》;
	//#	define EPSILON /*FLT_EPSILON*/1.192092896e-06
	//	static bool isSegmentsIntersect3(const osg::Vec3d& p1, const osg::Vec3d& q1, const osg::Vec3d& p2, const osg::Vec3d& q2, osg::Vec3d &p)
	//	{
	//		double s, t;
	//		osg::Vec3d c1, c2;
	//		osg::Vec3d d1 = q1 - p1;
	//		osg::Vec3d d2 = q2 - p2;
	//		osg::Vec3d r = p1 - p2;
	//		double a = d1* d1;
	//		double e = d2* d2;
	//		double f = d2* r;
	//		double res_dis = DBL_MAX;
	//
	//		if (a <= EPSILON && e <= EPSILON) {
	//			s = t = 0.0f;
	//			c1 = p1;
	//			c2 = p2;
	//			res_dis = (c1 - c2)*(c1 - c2);
	//		}
	//		else {
	//			if (a <= EPSILON) {
	//				s = 0.0f;
	//				t = f / e;
	//				t = osg::clampTo(t, 0.0, 1.0);
	//			}
	//			else {
	//				double c = (d1* r);
	//				if (e <= EPSILON) {
	//					t = 0.0f;
	//					s = osg::clampTo(-c / a, 0.0, 1.0);
	//				}
	//				else {
	//					double b = (d1* d2);
	//					double denom = a*e - b*b;
	//					if (denom != 0.0f) {
	//						s = osg::clampTo((b*f - c*e) / denom, 0.0, 1.0);
	//					}
	//					else s = 0.0f;
	//					t = (b*s + f) / e;
	//					if (t < 0.0f) {
	//						t = 0.0f;
	//						s = osg::clampTo(-c / a, 0.0, 1.0);
	//					}
	//					else if (t > 1.0f) {
	//						t = 1.0f;
	//						s = osg::clampTo((b - c) / a, 0.0, 1.0);
	//					}
	//				}
	//			}
	//			c1 = p1 + d1 * s;
	//			c2 = p2 + d2 * t;
	//			res_dis = (c1 - c2)*(c1 - c2);
	//		}
	//		if (res_dis < EPSILON) {
	//			p = c1;
	//			return true;
	//		}
	//		return false;
	//	}

		//static osg::Vec3dArray* getSubArray(const osg::Vec3dArray* va, int first, int last)
		//{
		//	if (!va || last < first) return 0L;

		//	int maxIndex = va->size() - 1;
		//	int f = osg::clampTo(first, 0, maxIndex);
		//	int l = osg::clampTo(last, 0, maxIndex);
		//	osg::ref_ptr<osg::Vec3dArray> subva = new osg::Vec3dArray;
		//	subva->reserve(l - f + 1);
		//	for (int j = f; j <= l; ++j)
		//	{
		//		subva->push_back(va->at(j));
		//	}
		//	return subva.release();
		//}

		//static osg::Vec3dArray* getSubArrayOfLoop(const osg::Vec3dArray* loopVa, int first, int last)
		//{
		//	int maxIndex = loopVa->size() - 1;
		//	int f = osg::clampTo(first, 0, maxIndex);
		//	int l = osg::clampTo(last, 0, maxIndex);
		//	osg::ref_ptr<osg::Vec3dArray> subva = new osg::Vec3dArray;
		//	if (f > l) {
		//		subva->reserve(maxIndex - f + l + 2);
		//		for (int i = f; i <= maxIndex; ++i)
		//			subva->push_back(loopVa->at(i));
		//		for (int j = 0; j <= l; ++j)
		//			subva->push_back(loopVa->at(j));
		//	}
		//	else {
		//		subva->reserve(l - f + 1);
		//		for (int i = f; i <= l; ++i)
		//			subva->push_back(loopVa->at(i));
		//	}
		//	return subva.release();
		//}

		//static osg::Vec3dArray* getReverseArray(const osg::Vec3dArray* va)
		//{
		//	osg::ref_ptr<osg::Vec3dArray> reVa = new osg::Vec3dArray;
		//	reVa->reserve(va->size());
		//	osg::Vec3dArray::const_reverse_iterator it;
		//	for (it = va->rbegin(); it != va->rend(); ++it)
		//	{
		//		reVa->push_back((*it));
		//	}
		//	return reVa.release();
		//}

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
		//for (it1 = va1->begin(), it2 = va2->begin(); it1 != va1->end(), it2 != va2->end(); ++it1, ++it2)
		//{
		//	if (!isVec3dEqual(*it1, *it2, epsilon))
		//	{
		//		return false;
		//	}
		//}
		//return true;
	}

	//static double getLineStripLength(const osg::Vec3dArray* va)
	//{
	//	double l = 0.0;
	//	for (osg::Vec3dArray::const_iterator it = va->begin() + 1; it != va->end(); ++it)
	//	{
	//		l += (*it - *(it - 1)).length();
	//	}
	//	return l;
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
}

////////////////////////////////////////////////////////////////////////////
////
//class Junction :public osg::Referenced
//{
//public:
//	Junction(const osg::Vec3d& p) :_position(p) {}
//
//	osg::Vec3d        _position; //交点的位置;
//	BooleanPathVector _paths;    //以_position为起点的关联路径链表
//};
//
//#define COMPUTE_BB_METHOD void computeBB() \
//{\
//	_bb.init();\
//	for (unsigned int i = 0; i < _va->size(); ++i)\
//	{\
//		_bb.expandBy(_va->at(i));\
//	}\
//}
//
////////////////////////////////////////////////////////////////////////////
//class Path : public osg::Referenced
//{
//public:
//	typedef std::pair<unsigned int, osg::ref_ptr<Junction> > JunctionPair;
//	typedef std::vector<JunctionPair> JunctionPairVector;
//
//public:
//	Path(osg::Vec3dArray* va) : _va(va), _isClosed(false)
//	{
//		if (va)
//		{
//			if (va->size() > 1)
//			{
//				if (isVec3dEqual(*(va->begin()), *(va->rbegin()), COMPUTEJUNCTION_EPSILON))
//				{
//					_isClosed = true;
//				}
//			}
//		}
//		computeBB();
//		_junctions.clear();
//	}
//
//	void addJunction(unsigned int i, Junction* junction)
//	{
//		if (!_junctions.empty())
//		{
//			JunctionPairVector::const_iterator it;
//			for (it = _junctions.begin(); it != _junctions.end(); ++it)
//			{
//				if ((*it).first == i)
//					return;
//			}
//		}
//		_junctions.push_back(JunctionPair(i, junction));
//	}
//
//private:
//	COMPUTE_BB_METHOD
//
//public:
//	std::string _name;
//	bool _isClosed;
//	osg::BoundingBox _bb;
//	JunctionPairVector _junctions;
//	osg::ref_ptr<osg::Vec3dArray> _va;
//};
//
////----------------------------------------------------------------------
//class BooleanPath :public osg::Referenced
//{
//public:
//	BooleanPath(osg::Vec3dArray* va)
//		: _isClosed(false)
//		, _va(va)
//		, _previous(0L)
//		, _next(0L)
//		, _twin(0L)
//		, _startJunction(0L)
//		, _endJunction(0L)
//	{}
//
//	bool & isClosed() { return _isClosed; }
//	const bool& isClosed() const { return _isClosed; }
//
//	osg::BoundingBox& boundingBox() { return _bb; }
//	const osg::BoundingBox& boundingBox() const { return _bb; }
//
//	osg::ref_ptr<Junction> &startJunction() { return _startJunction; }
//	const osg::ref_ptr<Junction> &startJunction() const { return _startJunction; }
//
//	osg::ref_ptr<Junction> &endJunction() { return _endJunction; }
//	const osg::ref_ptr<Junction> &endJunction() const { return _endJunction; }
//
//	osg::ref_ptr<BooleanPath>& previousPath() { return _previous; }
//	const osg::ref_ptr<BooleanPath>& previousPath() const { return _previous; }
//
//	osg::ref_ptr<BooleanPath>& nextPath() { return _next; }
//	const osg::ref_ptr<BooleanPath>& nextPath() const { return _next; }
//
//	osg::ref_ptr<BooleanPath>& twinPath() { return _twin; }
//	const osg::ref_ptr<BooleanPath>& twinPath() const { return _twin; }
//
//	osg::ref_ptr<osg::Vec3dArray>& pointArray() { return _va; }
//	const osg::ref_ptr<osg::Vec3dArray>& pointArray() const { return _va; }
//
//private:
//	COMPUTE_BB_METHOD
//
//private:
//	bool						  _isClosed;     //是否自闭合;
//	osg::BoundingBox			  _bb;			 //矩形包围框;
//	osg::ref_ptr<Junction>		  _startJunction;//起点;
//	osg::ref_ptr<Junction>		  _endJunction;  //终点;
//	osg::ref_ptr<BooleanPath>	  _previous;     //前一路径;
//	osg::ref_ptr<BooleanPath>	  _next;         //下一路径;
//	osg::ref_ptr<BooleanPath>	  _twin;         //对应的反向路径;
//	osg::ref_ptr<osg::Vec3dArray>  _va;			 //对应的参数曲线;
//public:
//	std::string _name;
//};
////////////////////////////////////////////////////////////////////////////
//
//struct DirectedLoop :public osg::Referenced
//{
//	enum Type { MAX_LOOP, MIN_LOOP, UNDEFINED };
//	DirectedLoop() :_type(UNDEFINED), _va(0L)
//	{
//		_booleanPathVector.clear();
//	}
//
//	Type& type() { return _type; }
//	const Type& type() const { return _type; }
//
//	osg::BoundingBox& boundingBox() { return _bb; }
//	const osg::BoundingBox& boundingBox() const { return _bb; }
//
//	BooleanPathVector& booleanPathVector() { return _booleanPathVector; }
//	const BooleanPathVector& booleanPathVector() const { return _booleanPathVector; }
//
//	osg::ref_ptr<osg::Vec3dArray>& pointArray() { return _va; }
//	const osg::ref_ptr<osg::Vec3dArray>& pointArray() const { return _va; }
//
//	bool isContains(const DirectedLoop& loop)
//	{
//		if (_bb.radius() < loop.boundingBox().radius()) return false;
//		if (!_bb.intersects(loop.boundingBox())) return false;
//		if (loop.pointArray()->size() < 3) return false;
//
//		//重合?（孤岛通常会出现这种情况）;
//		if (_booleanPathVector.size() == loop.booleanPathVector().size())
//		{
//			bool is_repeat = true;
//			BooleanPathVector::const_iterator it0, it1;
//			for (it0 = _booleanPathVector.begin(); it0 != _booleanPathVector.end(); ++it0)
//			{
//				bool isSamePath = false;
//				for (it1 = loop.booleanPathVector().begin(); it1 != loop.booleanPathVector().end(); ++it1)
//				{
//					if ((*it0)->twinPath() == *it1)
//					{
//						isSamePath = true;
//						break;
//					}
//				}
//				if (!isSamePath)
//				{
//					is_repeat = false;
//					break;
//				}
//			}
//			if (is_repeat) return false;
//		}
//
//		//求交点数量;--判断一个不黏连的点就够了（因为算法可以保证不会出现部分覆盖的情况）。
//		//也保证了每个环的顶点数量不能小于1（好像小于2比较合适O(∩_∩)O, 但是这不重要）;
//		osg::Vec3d dir = normalize(loop.pointArray()->at(0) - loop.pointArray()->at(1));
//		for (unsigned int i = 0; i < loop.pointArray()->size(); ++i)
//		{
//			const osg::Vec3d& p1 = loop.pointArray()->at(i);
//			if (isRepeat(_va, p1, GEO_EPSILOND))//选择不黏连的点;
//			{
//				continue;
//			}
//			int hitNum = 0;
//			osg::Vec3d p2 = p1 + dir*LARGE_DATA/*FLT_MAX*/;
//			for (unsigned int j = 0; j < _va->size() - 1; ++j)
//			{
//				osg::Vec3d p3 = _va->at(j);
//				osg::Vec3d p4 = _va->at(j + 1);
//				if (isVec3dEqual(p3, p4, GEO_EPSILOND))
//				{
//					continue;
//				}
//				if (isSegmentsIntersect3(p1, p2, p3, p4, osg::Vec3d()))
//				{
//					hitNum++;
//				}
//			}
//			{//首尾;
//				osg::Vec3d p3 = _va->at(0);
//				osg::Vec3d p4 = _va->at(_va->size() - 1);
//				if (!isVec3dEqual(p3, p4, GEO_EPSILOND))
//				{
//					if (isSegmentsIntersect3(p1, p2, p3, p4, osg::Vec3d()))
//					{
//						hitNum++;
//					}
//				}
//			}
//			if (hitNum % 2 == 0)
//			{
//				return false;
//			}
//			return true;
//		}
//		return false;
//	}
//
//	void dirty(const osg::Vec3d& clipPlaneNormal)
//	{
//		unsigned vertexnum = 0;
//		BooleanPathVector::const_iterator it0;
//		for (it0 = _booleanPathVector.begin(); it0 != _booleanPathVector.end(); ++it0)
//		{
//			vertexnum += (*it0)->pointArray()->size();
//		}
//		if (!_va) _va = new osg::Vec3dArray;
//		if (!_va->empty())_va->clear();
//		_va->reserve(vertexnum);
//		BooleanPathVector::const_iterator it;
//		for (it = _booleanPathVector.begin(); it != _booleanPathVector.end(); ++it)
//		{
//			osg::Vec3dArray::const_iterator vi;
//			for (vi = (*it)->pointArray()->begin(); vi != (*it)->pointArray()->end(); ++vi)
//			{
//				_va->push_back(*vi);
//			}
//		}
//		computeBB();
//		type() = isArrayInClockwiseDirection(_va, &_bb, clipPlaneNormal) ? MAX_LOOP : MIN_LOOP;
//	}
//private:
//	COMPUTE_BB_METHOD
//
//private:
//	Type _type;
//	osg::BoundingBox _bb;
//	BooleanPathVector _booleanPathVector;
//	osg::ref_ptr<osg::Vec3dArray> _va;
//};

//////////////////////////////////////////////////////////////////////////
namespace
{
	//static void removeBooleanPathFromJunction(BooleanPath* fpath)
	//{
	//	if (!fpath) return;
	//	Junction* sjunction = fpath->startJunction();
	//	if (!sjunction) return;
	//	if (sjunction->_paths.empty()) return;

	//	BooleanPathVector::iterator it;
	//	it = std::find(sjunction->_paths.begin(), sjunction->_paths.end(), fpath);
	//	if (it == sjunction->_paths.end())
	//	{
	//		return;
	//	}
	//	sjunction->_paths.erase(it);
	//}

	//static BooleanPath* getClockwiseNearstBooleanPath(const BooleanPathVector& paths, const BooleanPath* path, const osg::Vec3d& c, const osg::Vec3d&n)
	//{
	//	if (!path) return 0L;
	//	if (path->pointArray()->empty()) return 0L;
	//	if (paths.empty()) return 0L;

	//	osg::ref_ptr<BooleanPath> resultPath = 0L;
	//	double minClockwiseAngle = DBL_MAX;

	//	//为了减少误差，取距离junction较远的点-也可能造成其他不可知问题，但是暂时没有完美的方式解决;
	//	osg::Vec3d v0 = getNearestFarerThan<osg::Vec3dArray>(path->pointArray(), c, getLineStripLength(path->pointArray())*0.1);
	//	osg::Vec3d dir0 = normalize(v0 - c);
	//	for (BooleanPathVector::const_iterator it = paths.begin(); it != paths.end(); ++it)
	//	{
	//		if (path == *it) continue;
	//		osg::Vec3d v1 = getNearestFarerThan<osg::Vec3dArray>((*it)->pointArray(), c, getLineStripLength((*it)->pointArray())*0.1);
	//		osg::Vec3d dir1 = normalize(v1 - c);

	//		double cloclwiseangle = getClockwiseAngle(dir0, dir1, n);
	//		if (minClockwiseAngle > cloclwiseangle)
	//		{
	//			minClockwiseAngle = cloclwiseangle;
	//			resultPath = *it;
	//		}
	//	}
	//	return resultPath.release();
	//}

	//static bool hasBooleanPathDeleted(BooleanPath* path)
	//{
	//	Junction* startJunction = path->startJunction();
	//	if (startJunction->_paths.empty()) return true;

	//	BooleanPathVector::iterator it;
	//	it = std::find(startJunction->_paths.begin(), startJunction->_paths.end(), path);
	//	if (it != startJunction->_paths.end())
	//	{
	//		return false;
	//	}
	//	return true;
	//}

	//static Junction* getJunctionAt(const osg::Vec3d& p, const JunctionVector& joints)
	//{
	//	const osg::Vec3d& p1 = p;
	//	for (JunctionVector::const_iterator it = joints.begin(); it != joints.end(); ++it)
	//	{
	//		const osg::Vec3d& p0 = (*it)->_position;
	//		if (isVec3dEqual(p0, p1, GEO_EPSILOND))
	//		{
	//			return (*it);
	//		}
	//	}
	//	return 0L;
	//}
}

//////////////////////////////////////////////////////////////////////////
SliceOperator::SliceOperator(bool isOnEarth) :_isOnEarth(isOnEarth), _maxCrackSize(DBL_MAX) {}
SliceOperator::~SliceOperator() {}

//void SliceOperator::generateJunctions(Path * path, JunctionVector & joints)
//{
//	return;
//	if (!path) return;
//	if (path->_isClosed) return;
//
//	const osg::ref_ptr<osg::Vec3dArray>& va1 = path->_va;
//	for (int i = 0; i < (int)va1->size() - 1; ++i)
//	{
//		const osg::Vec3d& v1 = (*va1)[i];
//		for (int j = i + 1; j < (int)va1->size(); ++j)
//		{
//			const osg::Vec3d& v2 = (*va1)[j];
//			if (isVec3dEqual(v1, v2, COMPUTEJUNCTION_EPSILON/*GEO_EPSILOND*/))
//			{
//				if (abs(i - j) < 1) continue;
//
//				osg::Vec3d p = (v1 + v2)*0.5;
//				osg::ref_ptr<Junction> joint = /*0L;//*/ getJunctionAt(p, joints);
//				if (!joint)
//				{
//					joint = new Junction(p);
//					joints.push_back(joint);
//				}
//				path->addJunction(i, joint);
//				path->addJunction(j, joint);
//			}
//		}
//	}
//}

//namespace
//{
//	//struct IJunction
//	//{
//	//	IJunction(unsigned int ind1, unsigned int ind2, const osg::Vec3d& p)
//	//		: _ind1(ind1), _ind2(ind2), _position(p)
//	//	{}
//	//	osg::Vec3d _position;
//	//	unsigned int _ind1, _ind2;
//	//};
//	//typedef std::vector<IJunction> IJunctionVec;
//}

//
////
//void SliceOperator::generateJunctions(Path* p1, Path* p2, JunctionVector& joints)
//{
//	if (!p1 || !p2) return;
//	if (p1->_isClosed && p2->_isClosed)
//	{
//		return;
//	}
//
//#if 1
//	//IJunctionVec junVec;
//
//	const osg::ref_ptr<osg::Vec3dArray>& va1 = p1->_va;
//	const osg::ref_ptr<osg::Vec3dArray>& va2 = p2->_va;
//	for (osg::Vec3dArray::const_iterator it1 = va1->begin(); it1 != va1->end(); ++it1/*it1 += va1->size()*/)
//	{
//		const osg::Vec3d& v1 = *it1;
//		for (osg::Vec3dArray::const_iterator it2 = va2->begin(); it2 != va2->end(); ++it2 /*it2 += va2->size()*/)
//		{
//			if (it1 != va1->begin() && it1 != va1->end() - 1 && it2 != va2->begin() && it2 != va2->end() - 1)
//			{
//				continue;
//			}
//			const osg::Vec3d& v2 = *it2;
//			if (isVec3dEqual(v1, v2, COMPUTEJUNCTION_EPSILON/*GEO_EPSILOND*/))
//			{
//				osg::Vec3d p = (v1 + v2)*0.5;
//				//junVec.push_back(IJunction(it1 - p1->_va->begin(), it2 - p2->_va->begin(), p));
//				osg::ref_ptr<Junction> joint = getJunctionAt(p, joints);
//				if (!joint)
//				{
//					joint = new Junction(p);
//					joints.push_back(joint);
//				}
//				unsigned int i1 = it1 - p1->_va->begin();
//				unsigned int i2 = it2 - p2->_va->begin();
//				p1->addJunction(i1, joint);
//				p2->addJunction(i2, joint);
//				//break;
//			}
//		}
//	}
//
//	//if (junVec.size() > 2)
//	//{
//	//	for (IJunctionVec::const_iterator it = junVec.begin(); it != junVec.end(); ++it)
//	//	{
//	//		osg::ref_ptr<Junction> joint = getJunctionAt(it->_position, joints);
//	//		if (!joint)
//	//		{
//	//			joint = new Junction(it->_position);
//	//			joints.push_back(joint);
//	//		}
//	//		unsigned int i1 = it->_ind1;
//	//		unsigned int i2 = it->_ind2;
//	//		p1->addJunction(i1, joint);
//	//		p2->addJunction(i2, joint);
//	//	}
//	//}
//	//#else
//		////  [9/18/2017 BIGDOG]-只考虑首尾相连的情况(测试用的，这样不合理);
//		//osg::Vec3d p1Start = *(p1->_va->begin());
//		//osg::Vec3d p1End = *(p1->_va->rbegin());
//
//		//osg::Vec3d p2Start = *(p2->_va->begin());
//		//osg::Vec3d p2End = *(p2->_va->rbegin());
//
//		//if (isVec3dEqual(p1Start, p2Start, COMPUTEJUNCTION_EPSILON))
//		//{
//		//	osg::Vec3d p = (p1Start + p2Start)*0.5;
//		//	osg::ref_ptr<Junction> joint = getJunctionAt(p, joints);
//		//	if (!joint)
//		//	{
//		//		joint = new Junction(p);
//		//		joints.push_back(joint);
//		//	}
//		//	p1->addJunction(0, joint);
//		//	p2->addJunction(0, joint);
//		//}
//		//if (isVec3dEqual(p1Start, p2End, COMPUTEJUNCTION_EPSILON))
//		//{
//		//	osg::Vec3d p = (p1Start + p2End)*0.5;
//		//	osg::ref_ptr<Junction> joint = getJunctionAt(p, joints);
//		//	if (!joint)
//		//	{
//		//		joint = new Junction(p);
//		//		joints.push_back(joint);
//		//	}
//		//	p1->addJunction(0, joint);
//		//	p2->addJunction(p2->_va->size() - 1, joint);
//		//}
//
//		//if (isVec3dEqual(p1End, p2End, COMPUTEJUNCTION_EPSILON))
//		//{
//		//	osg::Vec3d p = (p1End + p2End)*0.5;
//		//	osg::ref_ptr<Junction> joint = getJunctionAt(p, joints);
//		//	if (!joint)
//		//	{
//		//		joint = new Junction(p);
//		//		joints.push_back(joint);
//		//	}
//		//	p1->addJunction(p1->_va->size() - 1, joint);
//		//	p2->addJunction(p2->_va->size() - 1, joint);
//		//}
//		//if (isVec3dEqual(p1End, p2Start, COMPUTEJUNCTION_EPSILON))
//		//{
//		//	osg::Vec3d p = (p1End + p2Start)*0.5;
//		//	osg::ref_ptr<Junction> joint = getJunctionAt(p, joints);
//		//	if (!joint)
//		//	{
//		//		joint = new Junction(p);
//		//		joints.push_back(joint);
//		//	}
//		//	p1->addJunction(p1->_va->size() - 1, joint);
//		//	p2->addJunction(0, joint);
//		//}
//		////if (isVec3dEqual(p1Start, p1End, COMPUTEJUNCTION_EPSILON))
//		////{
//		////	std::cout << "wa" << std::endl;
//		////}
//		////if (isVec3dEqual(p2Start, p2End, COMPUTEJUNCTION_EPSILON))
//		////{
//		////	std::cout << "wa" << std::endl;
//		////}
//	//#endif
//
//#else
//	for (unsigned int i = 0; i < p1->_va->size() - 1; ++i)
//	{
//		const osg::Vec3d& s1 = p1->_va->at(i);
//		const osg::Vec3d& e1 = p1->_va->at(i + 1);
//		for (unsigned int j = 0; j < p2->_va->size() - 1; ++j)
//		{
//			const osg::Vec3d& s2 = p2->_va->at(j);
//			const osg::Vec3d& e2 = p2->_va->at(j + 1);
//			osg::Vec3d p;
//			if (isSegmentsIntersect3(s1, e1, s2, e2, p))
//			{
//				if (p.isNaN()) continue;
//
//				osg::ref_ptr<Junction> joint = getJunctionAt(p, joints);
//				if (!joint)
//				{
//					joint = new Junction(p);
//					joints.push_back(joint);
//				}
//				//length
//				unsigned int ii = (p - s1).length2() < (p - e1).length2() ? i : i + 1;
//				unsigned int jj = (p - s2).length2() < (p - e2).length2() ? j : j + 1;
//
//				p1->addJunction(ii, joint);
//				p2->addJunction(jj, joint);
//			}
//		}
//	}
//#endif
//}
//
//bool compairJunction(Path::JunctionPair& j1, Path::JunctionPair& j2)
//{
//	return j1.first < j2.first;
//}
//
//void SliceOperator::generateJunctions(const Vec3dArrayVector& vas, const osg::Vec3d& planeNormal, JunctionVector& outJoints, PathVector& paths)
//{
//	unsigned pathIndex = 0;
//	/*PathVector paths;*/ paths.reserve(vas.size());
//	for (Vec3dArrayVector::const_iterator it = vas.begin(); it != vas.end(); ++it)
//	{
//		std::stringstream ss; ss << pathIndex << "_";
//		osg::ref_ptr<Path> path = new Path((*it));
//		path->_name = ss.str();
//		paths.push_back(path);
//		pathIndex++;
//	}
//
//	JunctionVector joints;
//	//自相交;
//	for (unsigned int i = 0; i < paths.size(); ++i)
//	{
//		generateJunctions(paths[i], joints);
//	}
//	//与其他线相交;
//	for (unsigned int i = 0; i < paths.size() - 1; ++i)
//	{
//		Path* path1 = paths[i];
//		if (!path1) continue;
//		for (unsigned int j = i + 1; j < paths.size(); ++j)
//		{
//			Path* path2 = paths[j];
//			if (!path2) continue;
//			if (!path1->_bb.intersects(path2->_bb))
//			{
//				continue;
//			}
//			generateJunctions(path1, path2, joints);
//		}
//	}
//
//	//生成关联路径;
//	BooleanPathVector bpvector;
//	bpvector.reserve(joints.size() * 4); joints.clear();
//	for (PathVector::const_iterator it = paths.begin(); it != paths.end(); ++it)
//	{
//		Path* path = *it;
//		removeContinuousRepeatJunctions(path);
//		generateBooleanPaths(path, bpvector);
//	}
//	outJoints.reserve(bpvector.size() * 2);
//	for (BooleanPathVector::const_iterator it = bpvector.begin(); it != bpvector.end(); ++it)
//	{
//		outJoints.push_back((*it)->startJunction());
//		outJoints.push_back((*it)->endJunction());
//	}
//}

////删除相邻的Junction（保留首尾），避免黏连情况;
//void SliceOperator::removeContinuousRepeatJunctions(Path* path)
//{
//	std::sort(path->_junctions.begin(), path->_junctions.end(), compairJunction);
//	if (path->_junctions.size() > 2)
//	{
//		Path::JunctionPairVector junctionsToRemoved;
//		Path::JunctionPairVector::iterator jit0 = path->_junctions.begin();
//		Path::JunctionPairVector::iterator jit1 = jit0 + 1;
//		Path::JunctionPairVector::iterator jit2 = jit1 + 1;
//		for (; jit2 != path->_junctions.end(); ++jit0, ++jit1, ++jit2)
//		{
//			if ((*jit1).first - (*jit0).first == 1 && (*jit2).first - (*jit1).first == 1)
//			{
//				junctionsToRemoved.push_back(*jit1);
//			}
//		}
//		if (path->_isClosed)
//		{
//			if (path->_junctions[0].first == 0 &&
//				path->_junctions[1].first == 1 &&
//				path->_junctions[path->_junctions.size() - 1].first == path->_va->size() - 1)
//			{
//				junctionsToRemoved.push_back(path->_junctions[0]);
//			}
//			if (path->_junctions[0].first == 0 &&
//				path->_junctions[path->_junctions.size() - 2].first == path->_va->size() - 2 &&
//				path->_junctions[path->_junctions.size() - 1].first == path->_va->size() - 1)
//			{
//				junctionsToRemoved.push_back(path->_junctions[path->_junctions.size() - 1]);
//			}
//		}
//		for (unsigned int i = 0; i < junctionsToRemoved.size(); ++i)
//		{
//			Path::JunctionPairVector::iterator jit;
//			jit = std::find(path->_junctions.begin(), path->_junctions.end(), junctionsToRemoved[i]);
//			if (jit != path->_junctions.end())
//			{
//				path->_junctions.erase(jit);
//			}
//		}
//	}
//}
//
//namespace
//{
//	bool isBooleanPathRepeat(const BooleanPathVector& paths, const BooleanPath* path)
//	{
//		for (BooleanPathVector::const_iterator it = paths.begin(); it != paths.end(); ++it)
//		{
//			if (path->isClosed())
//			{
//				if (!(*it)->isClosed()) continue;
//				if (isAlmostSameArray(path->pointArray(), (*it)->pointArray(), /*PATHREPEAT_EPSILON*/GEO_EPSILOND))
//				{
//					return true;
//				}
//			}
//			else
//			{
//				if ((*it)->isClosed()) continue;
//				if (path->startJunction() != (*it)->startJunction() &&
//					path->endJunction() != (*it)->endJunction())
//				{
//					continue;
//				}
//				if (isAlmostSameArray(path->pointArray(), (*it)->pointArray(), /*PATHREPEAT_EPSILON*/GEO_EPSILOND))
//				{
//					return true;
//				}
//			}
//		}
//		return false;
//	}
//}
//
//void SliceOperator::generateBooleanPaths(Path* path, BooleanPathVector &bpvector)
//{
//	if (!path) return;
//#if 0
//	if (path->_junctions.size() < 2)
//#else
//	if (path->_isClosed)
//#endif
//	{
//		//自己单独成闭环--如果junction的数量为1，忽略此junction;
//		osg::ref_ptr<osg::Vec3dArray> va0 = getSubArray(path->_va, 0, path->_va->size() - 1);
//		assert(va0); assert(!va0->empty());
//		if (va0 && !va0->empty())
//		{
//			//在任意位置添加两个junction，这里选起点和终点;
//			osg::ref_ptr<Junction> sjunction = new Junction(va0->at(0));
//			osg::ref_ptr<Junction> ejunction = new Junction(va0->at(va0->size() - 1));
//
//			osg::ref_ptr<BooleanPath> bp0 = new BooleanPath(va0);
//			bp0->startJunction() = sjunction;
//			bp0->endJunction() = ejunction;
//			bp0->isClosed() = true;
//			bp0->_name = path->_name + "0";
//			if (!isBooleanPathRepeat(bpvector, bp0))
//			{
//				osg::ref_ptr<osg::Vec3dArray> va1 = getReverseArray(va0);
//				osg::ref_ptr<BooleanPath> bp1 = new BooleanPath(va1);
//				bp1->startJunction() = ejunction;
//				bp1->endJunction() = sjunction;
//				bp1->isClosed() = true;
//				bp1->_name = path->_name + "00";
//
//				bp0->twinPath() = bp1;
//				bp1->twinPath() = bp0;
//				bpvector.push_back(bp0);
//				bpvector.push_back(bp1);
//				sjunction->_paths.push_back(bp0);
//				ejunction->_paths.push_back(bp1);
//			}
//		}
//	}
//	else
//	{
//		for (unsigned int i = 0; i < path->_junctions.size() - 1; ++i)
//		{
//			if (path->_junctions.size() < 2)
//			{
//				//OSG_NOTICE << "junctions size is: " << path->_junctions.size() << std::endl;
//				break;
//			}
//			Junction* currentJunction = path->_junctions[i].second;
//			Junction* nextJunction = path->_junctions[i + 1].second;
//
//			unsigned int currentVertexIndex = path->_junctions[i].first;
//			unsigned int nextVertexIndex = path->_junctions[i + 1].first;
//			if (nextVertexIndex == currentVertexIndex) continue;
//
//			bool isclosed = isVec3dEqual(path->_va->at(currentVertexIndex), path->_va->at(nextVertexIndex), GEO_EPSILOND);
//
//			//往后一个取值，确保不出现重复的点;
//			int firstindex = currentVertexIndex == path->_va->size() - 1 ? 0 : currentVertexIndex + 1;
//			osg::ref_ptr<osg::Vec3dArray> va0 = getSubArray(path->_va, firstindex, nextVertexIndex);
//			assert(va0); assert(!va0->empty());
//			if (!va0 || va0->empty()) continue;
//
//			std::stringstream ss; ss << i;
//			osg::ref_ptr<BooleanPath> bp0 = new BooleanPath(va0);
//			bp0->startJunction() = currentJunction;
//			bp0->endJunction() = nextJunction;
//			bp0->isClosed() = isclosed;
//
//			//if (isclosed)
//			//{
//			//	OSG_NOTICE << "closed" << std::endl;
//			//}
//
//			bp0->_name = path->_name + ss.str();
//			if (isBooleanPathRepeat(bpvector, bp0))
//			{
//				continue;
//			}
//
//			int lastindex = nextVertexIndex == 0 ? path->_va->size() - 1 : nextVertexIndex - 1;
//			osg::ref_ptr<osg::Vec3dArray> tempva1 = getSubArray(path->_va, currentVertexIndex, lastindex);
//			osg::ref_ptr<osg::Vec3dArray> va1 = getReverseArray(tempva1);
//			osg::ref_ptr<BooleanPath> bp1 = new BooleanPath(va1);
//			bp1->startJunction() = nextJunction;
//			bp1->endJunction() = currentJunction;
//			bp1->isClosed() = isclosed;
//
//			//if (isVec3dEqual(va1->front(), va1->back(), GEO_EPSILOND))
//			//{
//			//	OSG_NOTICE << "I know" << std::endl;
//			//}
//
//			bp1->_name = bp0->_name + ss.str();
//
//			bp0->twinPath() = bp1;
//			bp1->twinPath() = bp0;
//			bpvector.push_back(bp0);
//			bpvector.push_back(bp1);
//			currentJunction->_paths.push_back(bp0);
//			nextJunction->_paths.push_back(bp1);
//		}
//	}
//}
//
////递归遍历线段，得到一系列闭环;
//void SliceOperator::traversePath(BooleanPath* currentPath, DirectedLoopVector& mloops, const osg::Vec3d& planeNormal)
//{
//	if (!currentPath) return;
//	assert(!hasBooleanPathDeleted(currentPath));
//	if (currentPath->isClosed())//自成一环;
//	{
//		//一个点不能成环（好像两个点也不是环哈O(∩_∩)O。。。;
//		if (currentPath->pointArray()->size() < 3/*2*/)
//		{
//			removeBooleanPathFromJunction(currentPath);
//			return;
//		}
//		osg::ref_ptr<DirectedLoop> dloop = new DirectedLoop;
//		dloop->booleanPathVector().push_back(currentPath);
//		dloop->dirty(planeNormal);
//		mloops.push_back(dloop);
//
//		currentPath->pointArray()->setName(currentPath->_name);
//
//		//其实这个不删除也行，因为自成一环的肯定不会再次被遍历到;
//		removeBooleanPathFromJunction(currentPath);
//		return;
//	}
//
//	Junction* endjunction = currentPath->endJunction();
//	assert(endjunction);
//	if (!endjunction) return;
//	if (endjunction->_paths.empty())
//	{
//		BooleanPath* prePath = currentPath->previousPath();
//		removeBooleanPathFromJunction(currentPath);
//		traversePath(prePath, mloops, planeNormal);
//		return;
//	}
//
//	BooleanPath* twinOfCurrentPath = currentPath->twinPath();
//	assert(twinOfCurrentPath);
//	if (!twinOfCurrentPath) return;
//	BooleanPath* nextPath = getClockwiseNearstBooleanPath(endjunction->_paths, twinOfCurrentPath, endjunction->_position, planeNormal);
//	if (!nextPath)
//	{
//		//删除相关path;
//		BooleanPath* prePath = currentPath->previousPath();
//		removeBooleanPathFromJunction(currentPath);
//		traversePath(prePath, mloops, planeNormal);
//		return;
//	}
//	//强制设置nextPath能一定程度增强容错能力;
//	currentPath->nextPath() = nextPath;
//
//	if (nextPath->previousPath())
//	{
//		//构成了一个环，保存并删除相关path;
//		osg::ref_ptr<DirectedLoop> maxloop = new DirectedLoop;
//		BooleanPath* fpath = nextPath->nextPath();
//		while (fpath != nextPath)
//		{
//			assert(fpath);
//			fpath->pointArray()->setName(fpath->_name);
//			maxloop->booleanPathVector().push_back(fpath);
//			removeBooleanPathFromJunction(fpath);
//			fpath = fpath->nextPath();
//		}
//		assert(fpath);
//		fpath->pointArray()->setName(fpath->_name);
//		removeBooleanPathFromJunction(fpath);
//		maxloop->booleanPathVector().push_back(fpath);
//		maxloop->dirty(planeNormal);
//		//if (maxloop->pointArray()->size() < 3)
//		//{
//		//	OSG_NOTICE << "why" << std::endl;
//		//}
//		if (maxloop->pointArray()->size() > 3)
//		{
//			mloops.push_back(maxloop);
//		}
//
//		return;
//	}
//	nextPath->previousPath() = currentPath;
//	traversePath(nextPath, mloops, planeNormal);
//}

//void SliceOperator::traverseJunction(Junction* junction, DirectedLoopVector& mloops, const osg::Vec3d& planeNormal)
//{
//	if (junction->_paths.empty()) return;
//#if 0
//	for (unsigned int i = 0; i < junction->_paths.size(); ++i)
//	{
//		BooleanPath* currentPath = junction->_paths.at(i);
//		if (!currentPath) return;
//		traversePath(currentPath, mloops, planeNormal);
//	}
//#else
//	BooleanPath* currentPath = *(junction->_paths.begin());
//	if (!currentPath) return;
//	traversePath(currentPath, mloops, planeNormal);
//#endif
//}

//void SliceOperator::generateDirectedLoops(JunctionVector& joints, const osg::Vec3d& planeNormal, DirectedLoopVector& outLoops)
//{
//	if (joints.empty()) return;
//	for (JunctionVector::iterator it = joints.begin(); it != joints.end(); ++it)
//	{
//		Junction* junction = *it;
//		if (!junction) continue;
//		//先设置preious path和next path为空;
//		for (JunctionVector::iterator i = joints.begin(); i != joints.end(); ++i)
//		{
//			for (BooleanPathVector::iterator bi = (*i)->_paths.begin(); bi != (*i)->_paths.end(); ++bi)
//			{
//				(*bi)->previousPath() = 0L;
//				(*bi)->nextPath() = 0L;
//			}
//		}
//		traverseJunction(junction, outLoops, planeNormal);
//	}
//}

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

//bool loopCompair(osg::ref_ptr<DirectedLoop> l1, osg::ref_ptr<DirectedLoop> l2)
//{
//	return l1->boundingBox().radius() > l2->boundingBox().radius();
//}
//
//osg::Node* SliceOperator::generateProfileNode(osg::Geometry* ogeom, const DirectedLoopVector & loops, const osg::Vec3d& normal)
//{
//	if (loops.empty()) return 0L;
//
//	DirectedLoopVector maxLoops, minLoops;
//	for (DirectedLoopVector::const_iterator it = loops.begin(); it != loops.end(); ++it)
//	{
//		assert((*it)->type() != DirectedLoop::UNDEFINED);
//		if (!(*it)) continue;
//		if ((*it)->type() == DirectedLoop::MAX_LOOP)
//			maxLoops.push_back(*it);
//		else if ((*it)->type() == DirectedLoop::MIN_LOOP)
//			minLoops.push_back(*it);
//	}
//
//	if (maxLoops.size() != minLoops.size())
//	{
//		OSG_NOTICE << "max num: " << maxLoops.size() << std::endl
//			<< "min num: " << minLoops.size() << std::endl << std::endl;
//	}
//	osg::ref_ptr<osg::Group> root = new osg::Group;
//	DirectedLoopVector::const_iterator minIt;
//	for (minIt = minLoops.begin(); minIt != minLoops.end(); ++minIt)
//	{
//		DirectedLoop* minloop = *minIt;
//		if (!minloop) continue;
//
//		//判断本身是不是孔洞;
//		//bool isHole = false;
//		int outLoopNum = 0;
//		for (DirectedLoopVector::const_iterator iit = minLoops.begin(); iit != minLoops.end(); ++iit)
//		{
//			if (!(*iit)) continue;
//			if (minloop == *iit) continue;
//			if ((*iit)->isContains(*minloop))
//			{
//				outLoopNum++;
//				//isHole = true;
//				//break;
//			}
//		}
//		//if (outLoopNum > 1)
//		//{
//		//	OSG_NOTICE << outLoopNum << std::endl;
//		//}
//		if (outLoopNum % 2 == 1 /*isHole*/)
//		{
//			continue;
//		}
//
//		Vec3dArrayVector vas;
//		vas.push_back(minloop->pointArray());
//
//		DirectedLoopVector::const_iterator maxIt;
//		for (maxIt = maxLoops.begin(); maxIt != maxLoops.end(); ++maxIt)
//		{
//			DirectedLoop* maxloop = *maxIt;
//			if (!maxloop) continue;
//			//判断是否包含孔洞;
//			if (minloop->isContains(*maxloop))
//			{
//				vas.push_back(maxloop->pointArray());
//			}
//		}
//		unsigned int pn = 0;
//		for (unsigned int i = 0; i < vas.size(); ++i)
//		{
//			pn += vas[i]->size();
//		}
//
//		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
//		osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array; va->reserve(pn);
//		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array; ca->reserve(pn);
//		osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array; na->reserve(pn);
//		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
//		geode->addDrawable(geom);
//		geom->setVertexArray(va);
//		geom->setColorArray(ca);
//		geom->setNormalArray(na, osg::Array::BIND_PER_VERTEX);
//		geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
//
//		osg::Vec4 geoColor = getGeometryColor(ogeom);
//
//		unsigned int totNum = 0;
//		for (Vec3dArrayVector::const_iterator it = vas.begin(); it != vas.end(); ++it)
//		{
//			unsigned int vetNum = 0;
//			for (osg::Vec3dArray::const_iterator vaIt = (*it)->begin(); vaIt != (*it)->end(); ++vaIt)
//			{
//				va->push_back((*vaIt));
//				ca->push_back(geoColor);
//				na->push_back(normal);
//				totNum++; vetNum++;
//			}
//			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, totNum - vetNum, vetNum));
//		}
//
//		osgUtil::Tessellator tscx;
//		tscx.setTessellationNormal((normal));
//		tscx.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
//		tscx.setBoundaryOnly(false);
//		tscx.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
//		tscx.retessellatePolygons(*geom);
//
//		//osgUtil::TriStripVisitor tsv;
//		//tsv.stripify(*geom);
//
//		//得到原始Geometry材质
//		/*osg::Vec4 geoColor = getGeometryColor(ogeom);*/
//		osg::ref_ptr<osg::Texture2D> geoTexture = getGeometryTexture(ogeom);
//
//		geom->setUserData(new GeologicalMaterial(geoColor, geoTexture));
//
//		//osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
//		//ca->push_back(geoColor);
//		//geom->setColorArray(ca, osg::Array::BIND_OVERALL);
//
//		if (geoTexture)
//		{
//			osg::ref_ptr<osg::Vec2Array> ta = generateTextureCoord(normal, va);
//			if (ta)
//			{
//				geom->setTexCoordArray(0, ta);
//				geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, geoTexture);
//			}
//		}
//		root->addChild(geode);
//	}
//
//	if (fillRenderMode)//test
//	{
//		osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
//		pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
//		root->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
//		root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//	}
//
//	return root.release();
//}

namespace
{
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
		//osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
		//for (unsigned int i = 0; i < va->size(); ++i)
		//{
		//	ca->push_back(color);
		//}
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
		//if (intersection.matrix.valid())//因为是对Geometry操作，所以就不需要matrix了;
		//{
		osg::ref_ptr<osg::Vec3dArray> vva = new osg::Vec3dArray;
		for (auto pitr = intersection.polyline.begin(); pitr != intersection.polyline.end(); ++pitr)
		{
			osg::Vec3d p = intersection.matrix.valid() ? (*pitr) * (*intersection.matrix) : (*pitr);
			vva->push_back(p);
		}
		intersection.matrix = 0;
		//删除重复冗余的点;
		//removeRepeat<osg::Vec3dArray>(vva);//
		removeRepeatExt<osg::Vec3dArray>(vva);
		removeNextRepeatExt<osg::Vec3dArray>(vva);
		if (vva->empty()) continue;
		vaVector.push_back(vva);
		//OSG_NOTICE << vva->size() << std::endl;
		//}
	}

	Ex_arrangement arr;
	std::vector<Segment_2> segments;
	for (Vec3dArrayVector::const_iterator it = vaVector.begin(); it != vaVector.end(); ++it)
	{
		osg::Vec3dArray* va = *it;
		for (unsigned int i = 0; i < va->size() - 1; ++i)
		{
			osg::Vec3d v0 = va->at(i);
			osg::Vec3d v1 = va->at(i + 1);
			Point_2 p0(v0[0], v0[1]);
			Point_2 p1(v1[0], v1[1]);
			segments.push_back(Segment_2(p0, p1));
		}
	}

	CGAL::insert_curves(arr, segments.begin(), segments.end());

	unsigned int faceSize = arr.number_of_faces();
	if (faceSize == 0) return 0L;

	DLoops loops;
	Ex_arrangement::Face_const_iterator  fit;
	for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
	{
		if (fit == arr.unbounded_face()) continue;

		osg::ref_ptr<DLoop> loop = new DLoop;
		//得到外环;
		osg::ref_ptr<osg::Vec3dArray> loopVa = getCbbArray<Ex_arrangement>(fit->outer_ccb());
		loop->_va = loopVa;

		//得到孔洞;
		if (fit->number_of_holes() > 0)
		{
			Ex_arrangement::Hole_const_iterator hi;
			for (hi = fit->holes_begin(); hi != fit->holes_end(); ++hi)
			{
				osg::ref_ptr<osg::Vec3dArray> holeVa = getCbbArray<Ex_arrangement>(*hi);
				if (holeVa->empty()) continue;
				loop->_holes.push_back(holeVa);
			}
		}
		loops.push_back(loop);
	}

	osg::ref_ptr<osg::Geode> profileGeode = new osg::Geode;

	for (DLoops::const_iterator it = loops.begin(); it != loops.end(); ++it)
	{
		DLoop* loop = *it;
		if (!loop) continue;

		osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;

		osg::ref_ptr<osg::Geometry> profileGeom = new osg::Geometry;
		profileGeom->setVertexArray(va);

		osg::Vec4 geoColor = getGeometryColor(geom);
		ca->push_back(geoColor);
		profileGeom->setColorArray(ca, osg::Array::BIND_OVERALL);


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

		profileGeode->addDrawable(profileGeom);
	}


	if (fillRenderMode)//test
	{
		osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
		pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
		profileGeode->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		profileGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	}
	//const osg::Vec3d& slicePlnaeNormal = sliceplane.getNormal();
	//JunctionVector junctions;
	//PathVector paths;
	//generateJunctions(vaVector, slicePlnaeNormal, junctions, paths);
	//DirectedLoopVector loops;
	//generateDirectedLoops(junctions, slicePlnaeNormal, loops);

	//osg::ref_ptr<osg::Node> profileNode = generateProfileNode(geom, loops, slicePlnaeNormal);

	osg::ref_ptr<osg::Group> resultGroup = new osg::Group;
	resultGroup->addChild(profileGeode);

	if (showSlicePoint)
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
		//{//junction
		//	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
		//	for (JunctionVector::const_iterator it = junctions.begin(); it != junctions.end(); ++it)
		//	{
		//		va->push_back((*it)->_position);
		//	}
		//	mt->addChild(generatePoint(va, osg::Vec4(0, 0, 0, 1), 20.0f));
		//}
		//{//loops
		//	for (DirectedLoopVector::const_iterator it = loops.begin(); it != loops.end(); ++it)
		//	{
		//		if (it->get()->type() == DirectedLoop::MAX_LOOP)
		//		{
		//			continue;
		//		}
		//		osg::Vec4 color = osg::Vec4(0, 1, 0, 1);
		//		osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
		//		for (unsigned int i = 0; i < (*it)->pointArray()->size(); ++i)
		//		{
		//			va->push_back((*it)->pointArray()->at(i));
		//		}
		//		mt->addChild(generatePoint(va,/* osg::Vec4(0, 1, 0, 1)*/color, 9.50f));
		//	}
		//}
		resultGroup->addChild(mt);
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