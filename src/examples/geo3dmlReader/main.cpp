#include <osg/Group>
#include <osg/Material>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/ShadeModel>
#include <osg/LineSegment>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include "VoxelMeshClipper.h"
#include <osgAnimation/EaseMotion>

static const char*color_vertex =
{
	"void main()  					\n"
	"{								\n"
	"	gl_Position = ftransform(); \n"
	"}  							\n"
};

static const char* color_frag =
{
	"uniform float osg_FrameTime;				\n"
	"											\n"
	"void main()								\n"
	"{											\n"
	"	float a = fract(osg_FrameTime);			\n"
	"	gl_FragColor = vec4(1.0, a, 0.0, 0.1);	\n"
	"}											\n"
};

static void bloom_state(osg::StateSet*ss)
{

	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(new osg::Shader(osg::Shader::VERTEX, color_vertex));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, color_frag));

	if (!ss)ss = new osg::StateSet;
	ss->setAttributeAndModes(program);
}

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


class GeomVisitor2 :public osg::NodeVisitor
{
public:
	GeomVisitor2(const osg::Vec4& color, float rat)
		:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		, _rat(rat)
		, _color(color)
		, _scolor(osg::Vec4(1, 0, 0, 1))
	{}

	void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
			if (!geom) continue;

			osg::Vec4Array* ca = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
			if (!ca) continue;

			for (unsigned int j = 0; j < ca->size(); ++j)
			{
				osg::Vec4 newColor = _scolor + (_color - _scolor)*_rat;
				ca->at(j) = newColor;
			}
			ca->dirty();
		}
	}

private:
	float _rat;
	osg::Vec4 _color;
	osg::Vec4 _scolor;
};

class FGRedoutCallback : public osg::NodeCallback {
public:
	FGRedoutCallback()
	{
		_motion = new osgAnimation::InOutCubicMotion(0, 1, 1, osgAnimation::Motion::LOOP);
		_desColor = osg::Vec4(1, 1, 0, 1);
	}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		_motion->update(0.5);
		float aa = _motion->getValue();

		GeomVisitor2 gv2(_desColor, aa);
		node->accept(gv2);
	}
private:
	osg::Vec4 _desColor;
	osg::ref_ptr<osgAnimation::InOutCubicMotion> _motion;
};

//
//double Distance(const osg::Vec3d& a, const osg::Vec3d& b)
//{
//	return (a - b).length();
//}


//double DistanceLine(const osg::Vec3d& a, const osg::Vec3d& b, const osg::Vec3d& c)  // a和b是线段的两个端点， c是检测点
//{
//	osg::Vec3d ab = b - a;
//	osg::Vec3d ac = c - a;
//
//	double f = ab * ac;
//	if (f < 0) return (a - c).length();// Distance(a, c);
//
//	double d = ab * ab;
//	if (f > d) return (b - c).length();// Distance(a, c);
//
//	f = f / d;
//	osg::Vec3d D = a + ab*f;   // c在ab线段上的投影点
//
//	return (a - D).length();// Distance(a, D);
//}


//struct LineSegmentd
//{
//	LineSegmentd(const osg::Vec3d& s, const osg::Vec3d& e)
//		:_s(s), _e(e)
//	{}
//	osg::Vec3d _s, _e;
//};
//typedef std::vector<LineSegmentd> LineSegmentVec;


int main()
{
	//double dis = DistanceLine(osg::Vec3d(), osg::Vec3d(5, 5, 5), osg::Vec3d(7, 7, 7));
	//double dis2 = dis*dis;
	std::string fpath = "D:\\Geo3DGml_GIT\\Geo3DML\\data\\geo3dml_test_models\\cubeMode\\aa.xml";
	std::string newpath = "E:\\DATA\\GeoData\\Workspace\\500\\workspace.xml";
	std::string ffpath = "E:\\DATA\\VoxelData\\v1_all.osgb";
	osg::ref_ptr<osg::Node> geo3dmlNode = osgDB::readNodeFile(newpath);




	//geo3dmlNode->getOrCreateStateSet()->setAttributeAndModes(bloom_state());

	//geo3dmlNode->setUpdateCallback(new FGRedoutCallback());



	//osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	//pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	//geo3dmlNode->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);

	//osgDB::writeNodeFile(*geo3dmlNode, "test.osgb");

	//osg::Plane plane;
	//plane.getNormal();

	//osg::ComputeBoundsVisitor cbv;
	//geo3dmlNode->accept(cbv);

	//osg::BoundingBox bb = cbv.getBoundingBox();

	//osg::Vec3 planePoint = bb.center();
	//osg::Vec3 planeNormal = osg::Vec3(1, 1, 1);

	//GeomVisitor gv(planePoint, planeNormal);
	//geo3dmlNode->accept(gv);

	//osg::ref_ptr<osg::Node> pf = gv._root;
	//FlatVisitor fv(planePoint);
	//pf->accept(fv);
	//osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	//mt->addChild(pf);
	//mt->setMatrix(osg::Matrixd::translate(planePoint));

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(geo3dmlNode);
	//root->addChild(mt);

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