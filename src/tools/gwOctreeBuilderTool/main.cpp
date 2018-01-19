#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgUtil/PrintVisitor>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <iostream>
#include <fstream>
#include <sstream>

#include "OctreeBuilder.h"

static float randomValue(float min, float max)
{
	return (min + (float)rand() / (RAND_MAX + 1.0f) * (max - min));
}

static osg::Vec3 randomVector(float min, float max)
{
	return osg::Vec3(randomValue(min, max),
		randomValue(min, max),
		randomValue(min, max));
}

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
	std::string fpath = "D:\\Geo3DGml_GIT\\Geo3DML\\data\\geo3dml_test_models\\cubeMode\\aa.xml";
	std::string fpath0 = "E:/DATA/VoxelData/v1_all.osgb";

	osg::ref_ptr<osg::Node> voxelModel = osgDB::readNodeFile(fpath);

	osg::ComputeBoundsVisitor cbv;
	voxelModel->accept(cbv);
	osg::BoundingBox globalBound = cbv.getBoundingBox();
	osg::Vec3d bbMin = globalBound._min;
	osg::Vec3d bbMax = globalBound._max;
	osg::Vec3d diff = bbMax - bbMin;

	double halfMaxDif = max(diff[0], diff[1], diff[2])*0.5;

	osg::Vec3d center = (bbMin + bbMax)*0.5;

	osg::Vec3d newMin= center - osg::Vec3d(halfMaxDif, halfMaxDif, halfMaxDif);
	osg::Vec3d newMax= center + osg::Vec3d(halfMaxDif, halfMaxDif, halfMaxDif);

	globalBound.set(newMin, newMax);

	VoxelVisitor vv;
	voxelModel->accept(vv);

	const ElementInfoVec& globalElements = vv.getVoxelElements();

	OctreeBuilder octree;
	osg::ref_ptr<osg::Group> root = octree.build(0, globalBound, globalElements);

	std::ofstream out("octree_output.txt");
	PrintNameVisitor printer(out);
	root->accept(printer);

	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	return viewer.run();
}
