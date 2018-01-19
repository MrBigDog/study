#include <gwGeological/Cutter.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>

#include <osg/Quat>
#include <osg/Depth>
#include <osg/NodeVisitor>
#include <osg/StencilTwoSided>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>


int main()
{
	osg::ref_ptr<osg::Group> root = new osg::Group;

	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("E:\\DATA\\cow.ive");
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("D:/00gwV3/gwGeologicalUtil2017.10.15/build/src/examples/gwGeologicalTest/profileTest.osgb");

	osg::ComputeBoundsVisitor cbv;
	node->accept(cbv);

	osg::BoundingBox bb = cbv.getBoundingBox();

	osg::Plane plane(osg::Vec3d(0, -1, 0), bb.center());

	osg::Polytope pt;
	pt.add(plane);
	//pt.add(osg::Plane(osg::Vec3d(0, 0, 1), bb.center()));

	Cutter cutter(pt);
	node->accept(cutter);

	root->addChild(node);

	osgViewer::Viewer viewer;
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.getDatabasePager()->setTargetMaximumNumberOfPageLOD(1);
	viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.setSceneData(root);
	viewer.setUpViewInWindow(20, 20, 1000, 800);
	viewer.realize();

	while (!viewer.done())
	{
		viewer.frame();
	}
	viewer.run();

	return 0;
}