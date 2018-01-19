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

#include "adjust_near_far_plane.hpp"

#define ON_AND_PROTECTED  osg::StateAttribute::ON | osg::StateAttribute::PROTECTED
#define OFF_AND_PROTECTED osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED

class geoDataProcess :public osg::NodeVisitor
{
public:
	geoDataProcess() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{ }

	void apply(osg::Geode& geode)
	{
		while (geode.getNumDrawables() > 1)
		{
			geode.removeDrawable(geode.getDrawable(1));
		}
	}
};

class GeologicalDirectoryVisitor
{
public:
	GeologicalDirectoryVisitor::GeologicalDirectoryVisitor(osg::Group* root)
		: _root(root)
	{
		_ca = new osg::Vec4Array;
		_ca->push_back(osg::Vec4(1, 0, 0, 1));
		_ca->push_back(osg::Vec4(1, 1, 0, 1));
		_ca->push_back(osg::Vec4(0, 1, 0, 1));
		_index = 0;
	}

public:
	void GeologicalDirectoryVisitor::traverse(const std::string& path)
	{
		if (osgDB::fileType(path) == osgDB::DIRECTORY)
		{
			if (handleDir(path))
			{
				osgDB::DirectoryContents files = osgDB::getDirectoryContents(path);
				for (osgDB::DirectoryContents::const_iterator f = files.begin(); f != files.end(); ++f)
				{
					if (f->compare(".") == 0 || f->compare("..") == 0)
						continue;

					std::string filepath = osgDB::concatPaths(path, *f);
					traverse(filepath);
				}
			}
		}
		else if (osgDB::fileType(path) == osgDB::REGULAR_FILE)
		{
			handleFile(path);
		}
	}

private:
	void GeologicalDirectoryVisitor::handleFile(const std::string& filename)
	{
		std::string ext = osgDB::getLowerCaseFileExtension(filename);
		if (ext != "ive"&&ext != "osgb") return;

		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);
		//geoDataProcess gdp;
		//node->accept(gdp);
		if (!node) return;

		//std::string sname = osgDB::getNameLessExtension(filename);
		//osgDB::writeNodeFile(*node, sname + ".IVE");

		if (_root && _root != NULL)
		{
			_root->addChild(node);
		}
	}

	bool GeologicalDirectoryVisitor::handleDir(const std::string& path)
	{
		return true;
	}

private:
	osg::ref_ptr<osg::Group> _root;
	osg::ref_ptr<osg::Vec4Array> _ca;
	unsigned int _index;
};

class GeometryVisitor :public osg::NodeVisitor
{
public:
	GeometryVisitor(const osg::Vec3d& pos)
		:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		, _pos(pos)
	{}

	virtual void apply(osg::Geode& geode)
	{
		gwUtil::MeshConsolidator::run(geode);
		geode.dirtyBound();

		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
			if (!va) continue;

			for (osg::Vec3Array::iterator it = va->begin(); it != va->end(); ++it)
			{
				*it = *it - _pos;
			}

			geom->dirtyDisplayList();
			geom->dirtyBound();
		}
		geode.dirtyBound();
	}
private:
	osg::Vec3d _pos;
};


class MyGuiEventHandler :public osgGA::GUIEventHandler
{
public:
	MyGuiEventHandler(osg::Node* geoNode, osg::Group* root, const osg::Plane& plane)
		:_geoNode(geoNode), _isVisible(true), _root(root), _plane(plane)
	{
		//_profileRoot = new osg::Group;
		//if (_root)
		//{
		//	_root->addChild(_profileRoot);
		//	//_plane.set(osg::Vec4d(0, 1, 1, -0.0));//-489
		//	_plane.set(osg::Vec4d(0, 0, 1, -456));//-489
		//	SliceOperator so;
		//	osg::Node* profilenode = so.slice(_geoNode, _plane);
		//	if (profilenode)
		//	{
		//		if (_profileRoot->getNumChildren() == 0)
		//		{
		//			_profileRoot->addChild(profilenode);
		//		}
		//		else
		//		{
		//			_profileRoot->replaceChild(_profileRoot->getChild(0), profilenode);
		//		}
		//	}
		//}
	}

	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
		{
			if (ea.getKey() == 'r' || ea.getKey() == 'R')
			{
				if (_geoNode)
				{
					_isVisible = !_isVisible;
					_geoNode->setNodeMask(_isVisible);
				}
			}

			else if (ea.getKey() == 'a' || ea.getKey() == 'A')
			{
				if (_root&&_geoNode)
				{
					_geoNode->setNodeMask(true);
					osg::Vec4 planevec = _plane.asVec4();
					planevec[3] += 0.5;
					_plane.set(planevec);
					SliceOperator so;
					osg::Node* profilenode = so.slice(_geoNode, _plane);
					if (profilenode)
					{
						if (_profileRoot->getNumChildren() == 0)
						{
							_profileRoot->addChild(profilenode);
						}
						else
						{
							_profileRoot->replaceChild(_profileRoot->getChild(0), profilenode);
						}
						OSG_NOTICE << "plane: " << planevec[0] << ", " << planevec[1] << ", " << planevec[2] << ", " << planevec[3] << std::endl;
					}
					_geoNode->setNodeMask(false);
				}
			}
			else if (ea.getKey() == 'd' || ea.getKey() == 'D')
			{
				if (_root&&_geoNode)
				{
					_geoNode->setNodeMask(true);
					osg::Vec4 planevec = _plane.asVec4();
					planevec[3] -= 0.5;
					_plane.set(planevec);
					SliceOperator so;
					osg::Node* profilenode = so.slice(_geoNode, _plane);
					if (profilenode)
					{
						if (_profileRoot->getNumChildren() == 0)
						{
							_profileRoot->addChild(profilenode);
						}
						else
						{
							_profileRoot->replaceChild(_profileRoot->getChild(0), profilenode);
						}
						OSG_NOTICE << "plane: " << planevec[0] << ", " << planevec[1] << ", " << planevec[2] << ", " << planevec[3] << std::endl;
					}
					_geoNode->setNodeMask(false);
				}
			}
		}
		return false;
	}
private:
	bool _isVisible;
	osg::Plane _plane;
	osg::ref_ptr<osg::Node> _geoNode;
	osg::ref_ptr<osg::Group> _root;
	osg::ref_ptr<osg::Group> _profileRoot;
};


osg::Node* generateUnitWell(const osg::Vec3d& center, double radius, double length, const osg::Vec4& color, osg::Texture2D* tex = 0L, unsigned int segNum = 17)
{
	double half_length = length*0.5;
	double delta_angle = osg::PI*2.0 / (double)(segNum - 1);
	double delta_s = delta_angle*radius;

	osg::Vec3d bottomCenter = center - osg::Vec3d(0, 0, 1)*half_length;
	osg::Quat rot(delta_angle, osg::Vec3d(0, 0, 1));
	osg::Vec3d currentDir = osg::Vec3d(1, 0, 0);

	double sumS = 0.0;

	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec2Array> ta = new osg::Vec2Array;
	for (unsigned int i = 0; i < segNum; ++i)
	{
		currentDir = rot*currentDir;

		osg::Vec3d bottomPos = bottomCenter + currentDir*radius;
		osg::Vec3d topPos = bottomPos + osg::Vec3d(0, 0, 1)*length;
		va->push_back(bottomPos);
		va->push_back(topPos);

		osg::Vec3d normal = (currentDir); normal.normalize();
		na->push_back(normal);
		na->push_back(normal);

		sumS += delta_s;
		ta->push_back(osg::Vec2(0.0, sumS));
		ta->push_back(osg::Vec2(length, sumS));
	}

	osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
	ca->push_back(color);

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(va);
	geom->setTexCoordArray(0, ta);
	geom->setNormalArray(na, osg::Array::BIND_PER_VERTEX);
	geom->setColorArray(ca, osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 0, va->size()));
	if (tex)
	{
		geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
	}

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom);

	//osgDB::writeNodeFile(*geode, "dog.osgb");

	return geode.release();
}



int main()
{
	osg::ref_ptr<osg::Group> root = new osg::Group;

	osg::ref_ptr<osg::MatrixTransform> georoot = new osg::MatrixTransform;
	root->addChild(georoot);
	{//test
		osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
		pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
		//georoot->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		//georoot->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	}

	GeologicalDirectoryVisitor gdv(georoot);
	//gdv.traverse("E:/DATA/tfsw_17.9.21(1)/dxf/geo_data(2017.10.20)");
	gdv.traverse("E:/DATA/GeoData/ALLmodel/1cdjc_171122_dxf/1cdjc_171122_osgb");

	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/1_Q4.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/2_Q4.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/3_Q4.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/4_Q32.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/5_Q32.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/6_Q31.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/7_K1t-J.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/8_K2j.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/9_K2j.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/10_K2g.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/11_K2g.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/12_K2g.osgb"));
	//georoot->addChild(osgDB::readNodeFile("E:/DATA/tfsw_17.9.21(1)/osgb/13_J3S.osgb"));

	osg::ComputeBoundsVisitor cbv1;
	georoot->accept(cbv1);
	osg::BoundingBox bb1 = cbv1.getBoundingBox();

	osg::Vec3d cc = osg::Vec3d(415829.00000000000, 3365183.5000000000, 491.00000000000000);
	osg::Vec3d cc1 = bb1.center() + osg::Vec3d(0, 0, -50.0); //-145.0  -85.0

	osg::Plane plane(osg::Vec3d(0, 1, 0), bb1.center());
	osg::Plane plane1(osg::Vec3d(0, 0, 1), cc1);//-155.0出问题。
	//										   //osg::Plane plane1(0.0, 0.0, 1.0, -0.245);//-225.0 -245.0出问题。
	//										   //osg::Plane plane1(osg::Vec3d(0, 0, 1), osg::Vec3d());//-155.0出问题。

	//SliceOperator so;
	//osg::ref_ptr<osg::Node> profilenode1 = so.slice(georoot, plane);
	//if (profilenode1)
	//{
	//	//osgDB::writeNodeFile(*profilenode, "dogprofile.osgb");
	//	root->addChild(profilenode1);
	//}
	//osg::ref_ptr<osg::Node> profilenode2 = so.slice(georoot, plane1);
	//if (profilenode2)
	//{
	//	root->addChild(profilenode2);
	//}

	//root->addChild(so.slice(geonode, plane1, root));
	//root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	//georoot->setNodeMask(0);

	//osg::Vec3d ccc(419577, 3373026, 464);
	//{//钻孔;
	//	WellGenerator wg(false);
	//	osg::ref_ptr<osg::Node> well = wg(georoot, 10.0, ccc/*bb1.center() + osg::Vec3d(100, 100, (bb1.zMax() - bb1.zMin())*0.5)*/, 0.5);
	//	if (well)
	//	{
	//		root->addChild(well);
	//	}
	//}

	//float vwidth = 50.0f;
	//{//隧道;
	//	osg::ref_ptr<osg::Vec3dArray> va = new osg::Vec3dArray;
	//	va->push_back(osg::Vec3d((bb1.xMax() + bb1.xMin())*0.5, bb1.yMax(), (bb1.zMax() + bb1.zMin())*0.5 - 2300.0));
	//	va->push_back(osg::Vec3d((bb1.xMax() + bb1.xMin())*0.5, bb1.yMin(), (bb1.zMax() + bb1.zMin())*0.5 - 2300.0));
	//	//va->push_back(osg::Vec3d(bb.xMax(), (bb.yMax() + bb.yMin())*0.5, (bb.zMax() + bb.zMin())*0.5));
	//	//va->push_back(osg::Vec3d(bb.xMin(), (bb.yMax() + bb.yMin())*0.5, (bb.zMax() + bb.zMin())*0.5));

	//	//SquareTunelGenerator tg(false);
	//	//root->addChild(tg(georoot, va, vwidth, vwidth/**0.5*/, SquareTunelGenerator::PLANE_TOP));

	//	CircleTunelGenerator ctg(false);
	//	root->addChild(ctg(georoot, va, vwidth));

	//	TrenchGenerator tg(false);
	//	root->addChild(tg(georoot, va, vwidth, 80.0));
	//}

	//float modelSize = vwidth*2.5f;
	//osg::Light* myLight2 = new osg::Light;
	//myLight2->setLightNum(0);
	//myLight2->setPosition(osg::Vec4(bb.center(), 1.0f));
	//myLight2->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	//myLight2->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	//myLight2->setConstantAttenuation(1.0f);
	//myLight2->setLinearAttenuation(1.0f / modelSize);
	//myLight2->setQuadraticAttenuation(0.0f / osg::square(modelSize));

	{//基坑
		double zcenter = (bb1.zMax() + bb1.zMin())*0.5 - 2300.0;
		double x0 = bb1.xMin() + (bb1.xMax() - bb1.xMin())*0.35;
		double x1 = bb1.xMin() + (bb1.xMax() - bb1.xMin())*0.65;
		double y0 = bb1.yMin() + (bb1.yMax() - bb1.yMin())*0.35;
		double y1 = bb1.yMin() + (bb1.yMax() - bb1.yMin())*0.65;
		osg::ref_ptr<osg::Vec3Array> va1 = new osg::Vec3Array;
		va1->push_back(osg::Vec3d(x0, y0, zcenter));
		va1->push_back(osg::Vec3d(x1, y0, zcenter));
		va1->push_back(osg::Vec3d(x1, y1, zcenter));
		va1->push_back(osg::Vec3d(x0, y1, zcenter));

		FoundationDitchGenerator fdg;
		root->addChild(fdg(georoot, va1));
	}


	osgViewer::Viewer viewer;
	//viewer.setLight(myLight2);
	//viewer.setLightingMode(osg::View::SKY_LIGHT);
	//osg::Light* light = viewer.getCamera()->getView()->getLight();
	//if (light != NULL)
	//{
	//	light->setAmbient(osg::Vec4(0.5, 0.5, 0.5, 1));
	//	light->setDiffuse(osg::Vec4(1, 1, 1, 1));
	//	light->setSpecular(osg::Vec4(1, 1, 1, 1));
	//	//light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
	//	light->setConstantAttenuation(0.0000001);
	//	//light->setLinearAttenuation(0.001);
	//	//light->setQuadraticAttenuation(0.00001);
	//	//light->setQuadraticAttenuation(0.00001);
	//	light->setPosition(osg::Vec4(bb.center(), 1.0));
	//	light->setDirection(osg::Vec3(0, 1, 0));
	//	//light->setSpotCutoff(80.0);
	//	//light->setSpotExponent(100.0);

	//	//osg::ref_ptr<osg::LightSource> ls = new osg::LightSource();
	//	//osg::ref_ptr< osg::Light> lt = new osg::Light;
	//	//lt->setLightNum(0);
	//	//lt->setPosition(osg::Vec4(0.0, -1.0, 0.0, 0));
	//	//lt->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.0));
	//	//lt->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	//	//ls->setLight(light);
	//}

	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::StatsHandler());
	//viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	//viewer.addEventHandler(new osgViewer::ThreadingHandler());
	//viewer.addEventHandler(new osgViewer::LODScaleHandler());
	//viewer.addEventHandler(new osgViewer::RecordCameraPathHandler());
	//viewer.addEventHandler(new osgViewer::ScreenCaptureHandler());
	viewer.addEventHandler(new MyGuiEventHandler(georoot, root, plane1));
	viewer.getDatabasePager()->setTargetMaximumNumberOfPageLOD(1);
	viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.setSceneData(root);
	viewer.setUpViewInWindow(20, 20, 1000, 800);

	viewer.getCamera()->addCullCallback(new AutoClipPlaneCullCallback);
	viewer.realize();

	while (!viewer.done())
	{
		viewer.frame();
	}
	viewer.run();

	return 0;
}