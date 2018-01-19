#include <osg/Group>
#include <osg/Material>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/ShadeModel>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include "VoxelMeshClipper.h"

void setTransparent(osg::StateSet* state, float alf)
{
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	state->setMode(GL_BLEND, osg::StateAttribute::ON);
	state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::BlendColor> bc = new osg::BlendColor(osg::Vec4(1.0, 1.0, 1.0, 0.0));
	osg::ref_ptr<osg::BlendFunc>bf = new osg::BlendFunc();
	state->setAttributeAndModes(bf, osg::StateAttribute::ON);
	state->setAttributeAndModes(bc, osg::StateAttribute::ON);
	bf->setSource(osg::BlendFunc::CONSTANT_ALPHA);
	bf->setDestination(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
	bc->setConstantColor(osg::Vec4(1, 1, 1, 0.5));
}

void writeToLocal(osg::Node* node, const std::string& name = "geo3dml.osg")
{
	osgDB::writeNodeFile(*node, name);
}


class GeomVisitor :public osg::NodeVisitor
{
public:
	GeomVisitor(const osg::Vec3& pp, const osg::Vec3& pn)
		:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		, _planePoint(pp)
		, _planeNormal(pn)
	{
		_root = new osg::Group;
		osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
		pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
		_root->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		_root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		osg::ShadeModel* shadeModel = new osg::ShadeModel;
		shadeModel->setDataVariance(osg::Object::STATIC);
		shadeModel->setMode(osg::ShadeModel::SMOOTH);
		_root->getOrCreateStateSet()->setAttribute(shadeModel);
	}

	void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
			if (!geom) continue;

			gwUtil::VolelMeshClipper vmc;
			osg::ref_ptr<osg::Node> profileNode = vmc.clip(geom, _planePoint, _planeNormal);
			if (profileNode.valid())
			{
				_root->addChild(profileNode);
			}
		}
	}

	osg::Vec3 _planePoint;
	osg::Vec3 _planeNormal;
	osg::ref_ptr<osg::Group> _root;
};

class FlatVisitor :public osg::NodeVisitor
{
public:
	FlatVisitor(const osg::Vec3& offset) :osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _offset(offset)
	{}

	void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
			if (!geom) continue;

			osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
			if (!va) continue;

			for (unsigned int vi = 0; vi < va->size(); ++vi)
			{
				va->at(vi) -= _offset;
			}
		}
	}

private:
	osg::Vec3 _offset;
};

int main()
{
	osg::ref_ptr<osg::Node> geo3dmlNode = osgDB::readNodeFile("D:\\Geo3DGml_GIT\\Geo3DML\\data\\geo3dml_test_models\\cubeMode\\aa.xml");
	//osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	//pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	//geo3dmlNode->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);

	//osgDB::writeNodeFile(*geo3dmlNode, "test.osgb");

	osg::ComputeBoundsVisitor cbv;
	geo3dmlNode->accept(cbv);

	osg::BoundingBox bb = cbv.getBoundingBox();

	osg::Vec3 planePoint = bb.center();
	osg::Vec3 planeNormal = osg::Vec3(1, 1, 1);

	GeomVisitor gv(planePoint, planeNormal);
	geo3dmlNode->accept(gv);

	osg::ref_ptr<osg::Node> pf = gv._root;
	FlatVisitor fv(planePoint);
	pf->accept(fv);
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->addChild(pf);
	mt->setMatrix(osg::Matrixd::translate(planePoint));

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(geo3dmlNode);
	root->addChild(mt);

	osgViewer::Viewer viewer;
	viewer.setSceneData(root);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.addEventHandler(new osgViewer::ThreadingHandler());
	viewer.addEventHandler(new osgViewer::LODScaleHandler());
	viewer.realize();
	viewer.run();

	return 0;
}