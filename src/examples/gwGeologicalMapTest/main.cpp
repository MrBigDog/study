#include <gwGeological/MapNode.h>
#include <gwGeological/SliceOperator.h>
#include <gwGeological/WellGenerator.h>
#include <gwGeological/TunnelGenerator.h>
#include <gwGeological/FoundationDitchGenerator.h>
#include <gwGeologicalUtil/MeshConsolidator.h>

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
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("E:/DATA/VoxelData/v1_all.osgb");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("D:/Geo3DGml_GIT/Geo3DMLBuild/src/examples/geo3dmlReader/geoModel.osg");
	if (!node.valid())
	{
		return 1;
	}
	////Õº≤„;
	osg::ref_ptr<GeologicalLayer0> glayer = new GeologicalLayer0("l1", node);

	//∆ √ÊÕº≤„;
	PlaneProfileOptions planeoptions;
	planeoptions.addPlane(osg::Plane(osg::Vec3(-1, -1, 0), osg::Vec3d(281.07714843750000 + 100.0, 589.49401855468750, -18.289400100708008)));
	osg::ref_ptr<LayerProfile> profileLayer = new LayerProfile("test", planeoptions);

	//glayer->addProfile(profileLayer);

	//MapÃÌº”Õº≤„;
	osg::ref_ptr<Map> map = new Map;
	map->addGeologicalLayer(glayer);

	////Map Node
	osg::ref_ptr<MapNode> mapnode = new MapNode(map);

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(mapnode);

	osgViewer::Viewer viewer;
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::StatsHandler());
	//viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	//viewer.addEventHandler(new osgViewer::ThreadingHandler());
	//viewer.addEventHandler(new osgViewer::LODScaleHandler());
	//viewer.addEventHandler(new osgViewer::RecordCameraPathHandler());
	//viewer.addEventHandler(new osgViewer::ScreenCaptureHandler());
	//viewer.addEventHandler(new MyGuiEventHandler(georoot, root, plane1));
	viewer.getDatabasePager()->setTargetMaximumNumberOfPageLOD(1);
	viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.setSceneData(root);
	viewer.realize();

	while (!viewer.done())
	{
		viewer.frame();
	}
	viewer.run();

	return 0;
}