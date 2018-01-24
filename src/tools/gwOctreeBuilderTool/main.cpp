#include <osg/Material>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/LineSegment>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgUtil/PrintVisitor>
#include <osgAnimation/EaseMotion>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <iostream>
#include <fstream>
#include <sstream>

#include "OctreeBuilder.h"
#include "EffectUtil.h"
#include "VoxelUtil.h"
#include <gwGeological/TunnelGenerator.h>


template<typename T>
static T min(const T& a, const T& b)
{
	return a < b ? a : b;
}

template<typename T>
static T min(const T& a, const T& b, const T& c)
{
	return min(min(a, b), c);
}

template<typename T>
static T min(const T& a, const T& b, const T& c, const T& d)
{
	return min(min(min(a, b), c), d);
}

template<typename T>
static T max(const T& a, const T& b)
{
	return a > b ? a : b;
}

template<typename T>
static T max(const T& a, const T& b, const T& c)
{
	return max(max(a, b), c);
}

template<typename T>
static T max(const T& a, const T& b, const T& c, const T& d)
{
	return max(max(max(a, b), c), d);
}

class PrintNameVisitor : public osgUtil::PrintVisitor
{
public:
	PrintNameVisitor(std::ostream& out) : osgUtil::PrintVisitor(out) {}

	void apply(osg::Node& node)
	{
		if (!node.getName().empty())
		{
			output() << node.getName() << std::endl;
			enter();
			traverse(node);
			leave();
		}
		else osgUtil::PrintVisitor::apply(node);
	}
};


class GeometryVisitor :public osg::NodeVisitor
{
public:
	GeometryVisitor(const osg::Vec3d& pos) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _pos(pos)
	{
	}

	void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
			if (!geom) continue;

			osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
			if (!va) continue;

			for (unsigned int j = 0; j < va->size(); ++j)
			{
				va->at(j) -= _pos;
			}

			geom->dirtyBound();
		}
	}

private:
	osg::Vec3d _pos;
};

class VoxelVisitor :public osg::NodeVisitor
{
public:
	VoxelVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{
		_voxelNum = 0;
	}

	void apply(osg::Geode& geode)
	{
		osg::MatrixList ml = geode.getWorldMatrices();

		osg::ComputeBoundsVisitor cbv;
		geode.accept(cbv);
		const osg::BoundingBox& bb = cbv.getBoundingBox();

		osg::BoundingBox newbb;
		newbb.set(bb._min*ml[0], bb._max*ml[0]);

		std::string name = geode.getName();
		if (name.empty()) name = "Voxel";

		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		mt->setMatrix(ml[0]);
		mt->addChild(&geode);

		std::stringstream ss; ss << name << ++_voxelNum;
		_voxelElements.push_back(OctreeBuilder::ElementInfo(ss.str(), newbb, mt));

		//osg::NodeVisitor::traverse(geode);
	}

	const ElementInfoVec& getVoxelElements() const { return _voxelElements; }

private:
	unsigned int _voxelNum;
	ElementInfoVec _voxelElements;
};


//
int main(int argc, char** argv)
{
	osg::ref_ptr<osg::Group> root = new osg::Group;

	std::string fpath = "D:\\Geo3DGml_GIT\\Geo3DML\\data\\geo3dml_test_models\\cubeMode\\aa.xml";
	std::string fpath0 = "E:/DATA/VoxelData/v1_all.osgb";

	osg::ref_ptr<osg::Node> voxelModel = osgDB::readNodeFile(fpath);
	osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
	voxelModel->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
	//setTransparent(voxelModel->getOrCreateStateSet(), 0.2);
	root->addChild(voxelModel);

	osg::ComputeBoundsVisitor cbv;
	voxelModel->accept(cbv);
	osg::BoundingBox globalBound = cbv.getBoundingBox();
	osg::Vec3d bbMin = globalBound._min;
	osg::Vec3d bbMax = globalBound._max;
	osg::Vec3d diff = bbMax - bbMin;

	double halfMaxDif = max(diff[0], diff[1], diff[2])*0.5;

	osg::Vec3d center = (bbMin + bbMax)*0.5;

	GeometryVisitor gvv(center);
	voxelModel->accept(gvv);

	osg::ComputeBoundsVisitor cbvv;
	voxelModel->accept(cbvv);

	globalBound = cbvv.getBoundingBox();

	osg::Plane plane(osg::Vec3d(0, 1, 0), center);
	osg::ref_ptr<osg::Vec3dArray> va = new osg::Vec3dArray;
	//va->push_back(osg::Vec3d((globalBound.xMax() + globalBound.xMin())*0.5, globalBound.yMax(), (globalBound.zMax() + globalBound.zMin())*0.5));
	//va->push_back(osg::Vec3d((globalBound.xMax() + globalBound.xMin())*0.5, globalBound.yMin(), (globalBound.zMax() + globalBound.zMin())*0.5));
	va->push_back(osg::Vec3d(globalBound.xMax(), (globalBound.yMax() + globalBound.yMin())*0.5, (globalBound.zMax() + globalBound.zMin())*0.5));
	va->push_back(osg::Vec3d(globalBound.xMin(), (globalBound.yMax() + globalBound.yMin())*0.5, (globalBound.zMax() + globalBound.zMin())*0.5));

	CircleTunelGenerator ctg(false);
	root->addChild(ctg(voxelModel, va, 2.5));

	std::vector<osg::ref_ptr<osg::Node> > nodeInTunel;
	getVoxelInTunel(va, 2.5, voxelModel, nodeInTunel);

	std::cout << nodeInTunel.size() << std::endl;

	osg::ref_ptr<osg::Group> tunelRoot = new osg::Group;

	for (unsigned int i = 0; i < nodeInTunel.size(); ++i)
	{
		tunelRoot->addChild(nodeInTunel[i]);
	}
	//tunelRoot->setUpdateCallback(new FGRedoutCallback());
	setBloomEffect3(tunelRoot, osg::Vec4(1, 1, 0, 1), osg::Vec4(1, 0, 0, 1));
	root->addChild(tunelRoot);

	//bloom_state(tunelRoot->getOrCreateStateSet());

	//osg::Vec3d newMin = center - osg::Vec3d(halfMaxDif, halfMaxDif, halfMaxDif);
	//osg::Vec3d newMax = center + osg::Vec3d(halfMaxDif, halfMaxDif, halfMaxDif);

	//globalBound.set(newMin, newMax);

	//VoxelVisitor vv;
	//voxelModel->accept(vv);

	//const ElementInfoVec& globalElements = vv.getVoxelElements();

	//OctreeBuilder octree;
	//osg::ref_ptr<osg::Group> root = octree.build(0, globalBound, globalElements);

	//std::ofstream out("octree_output.txt");
	//PrintNameVisitor printer(out);
	//root->accept(printer);

	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	return viewer.run();
}
