#include <osg/Quat>
#include <osg/Depth>
#include <osg/NodeVisitor>
#include <osg/StencilTwoSided>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <gwGeologicalUtil/Random.h>

class VBOVisitor :public osg::NodeVisitor
{
public:
	VBOVisitor() :osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{}

	virtual void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			//osg::Vec4 color = osg::Vec4(_random.next(), _random.next(), _random.next(), 1.0);
			//osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
			//ca->push_back(color);

			//geom->setColorArray(ca, osg::Array::BIND_OVERALL);
			geom->setUseDisplayList(false);
			geom->setUseVertexBufferObjects(true);
		}
	}
private:
	gwUtil::Random _random;
};

//class ColorVisitor :public osg::NodeVisitor
//{
//public:
//	ColorVisitor(const osg::Vec4& color)
//		:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
//		, _color(color)
//	{}
//
//	virtual void apply(osg::Geode& geode)
//	{
//		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
//		{
//			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
//			if (!geom) continue;
//
//			osg::Vec4 color = osg::Vec4(_random.next(), _random.next(), _random.next(), 1.0);
//
//			osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
//			ca->push_back(color);
//
//			geom->setColorArray(ca, osg::Vec4Array::BIND_OVERALL);
//		}
//	}
//private:
//	osg::Vec4 _color;
//	gwUtil::Random _random;
//};

class GeologicalDirectoryVisitor
{
public:
	GeologicalDirectoryVisitor::GeologicalDirectoryVisitor(osg::Group* root)
		: _root(root)
	{}

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
#if 0
		if (ext != "dxf") return;

		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);
		if (!node) return;

		VBOVisitor vbov;
		node->accept(vbov);

		std::string sname = osgDB::getNameLessExtension(filename);
		osgDB::writeNodeFile(*node, sname + ".osgb", new osgDB::Options("WriteImageHint=IncludeData Compressor=zlib"));
#else
		//if (ext != "osgb") return;
		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);
		if (!node) return;
		_root->addChild(node);
#endif
	}

	bool GeologicalDirectoryVisitor::handleDir(const std::string& path)
	{
		return true;
	}

private:
	osg::ref_ptr<osg::Group> _root;
};


int main()
{
	osg::ref_ptr<osg::Group> root = new osg::Group;

	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("E:/DATA/GeoData/ALLmodel/dem.ive");

	VBOVisitor vbov;
	node->accept(vbov);

	osgDB::writeNodeFile(*node, "mybigdog.osgb", new osgDB::Options("WriteImageHint=IncludeData Compressor=zlib"));



	//GeologicalDirectoryVisitor gdv(root);
	//gdv.traverse("E:\\DATA\\GeoData\\ALLmodel\\5dxq_171110_dxf\\dxq_dxf_171110");

	//VBOVisitor vbov;
	//root->accept(vbov);

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