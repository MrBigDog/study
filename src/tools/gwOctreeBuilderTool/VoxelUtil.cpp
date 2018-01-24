#include "VoxelUtil.h"
#include <osg/Geode>
#include <osg/NodeVisitor>
#include <osg/ComputeBoundsVisitor>

namespace
{
	struct LineSegmentd
	{
		LineSegmentd(const osg::Vec3d& s, const osg::Vec3d& e)
			:_s(s), _e(e)
		{}
		osg::Vec3d _s, _e;
	};
	typedef std::vector<LineSegmentd> LineSegmentVec;


	double distPointToSegment(const osg::Vec3d& P, LineSegmentd S)
	{
		osg::Vec3d v = S._e - S._s;
		osg::Vec3d w = P - S._s;

		double c1 = w*v;
		if (c1 <= 0) return (P - S._s).length();

		double c2 = v* v;
		if (c2 <= c1) return (P - S._e).length();

		double b = c1 / c2;
		osg::Vec3d Pb = S._s + v*b;

		return (P - Pb).length();
	}

	class GeomVisitor3 : public osg::NodeVisitor
	{
	public:
		GeomVisitor3(const LineSegmentVec& lineSegs, float r)
			:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _lineSegs(lineSegs)
			, _r(r)
		{}

		void apply(osg::Geode& geode)
		{
			const osg::MatrixList& ml = geode.getWorldMatrices();
			const osg::Matrixd& mat = ml[0];
			osg::Matrixd invmat = osg::Matrixd::inverse(mat);

			osg::ComputeBoundsVisitor cbv;
			geode.accept(cbv);
			const osg::BoundingBox& bb = cbv.getBoundingBox();

			for (unsigned int i = 0; i < _lineSegs.size(); ++i)
			{
				LineSegmentd ls = _lineSegs[i];
				if (!invmat.isIdentity())
				{
					ls._s = ls._s*invmat;
					ls._e = ls._e*invmat;
				}

				for (unsigned int j = 0; j < 8; ++j)
				{
					double dis = distPointToSegment(bb.corner(j), ls);
					if (dis < _r)
					{
						_result.push_back(&geode);
						break;
					}
				}
			}
		}

		float _r;
		LineSegmentVec _lineSegs;
		std::vector<osg::ref_ptr<osg::Node> > _result;
	};
}


void getVoxelInTunel(osg::Vec3dArray* va, float r, osg::Node* geoModel, std::vector<osg::ref_ptr<osg::Node> >& outResult)
{
	if (!va) return;

	LineSegmentVec lsv; lsv.reserve(va->size() - 1);
	for (unsigned int i = 1; i < va->size(); ++i)
	{
		osg::Vec3d v1 = va->at(i);
		osg::Vec3d v0 = va->at(i - 1);
		LineSegmentd ls(v0, v1);
		lsv.push_back(ls);
	}

	GeomVisitor3 gv(lsv, r);
	geoModel->accept(gv);

	for (unsigned int i = 0; i < gv._result.size(); ++i)
	{
		outResult.push_back(gv._result[i]);
	}
}