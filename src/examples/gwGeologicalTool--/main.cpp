////#include "GeologicalProcessor.h"
//
//#include <osgDB/ReadFile>
//#include <osgUtil/Optimizer>
//#include <osgUtil/Simplifier>
//#include <osgViewer/Viewer>
//#include <osgGA/TrackballManipulator>
//#include <osgGA/StateSetManipulator>
//#include <osgViewer/ViewerEventHandlers>
//#include <iostream>
//#include <osgUtil/MeshOptimizers>
//
//#include <osgEarth/QuadTree>
//

//
//class KeyboardEventHandler : public osgGA::GUIEventHandler
//{
//public:
//	KeyboardEventHandler(unsigned int& flag) : _flag(flag) {}
//	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
//	{
//		switch (ea.getEventType())
//		{
//		case(osgGA::GUIEventAdapter::KEYDOWN):
//		{
//			if (ea.getKey() == 'n')
//			{
//				_flag = 1;
//				return true;
//			}
//			if (ea.getKey() == 'p')
//			{
//				_flag = 2;
//				return true;
//			}
//			break;
//		}
//		default:
//			break;
//		}
//		return false;
//	}
//
//private:
//	unsigned int& _flag;
//};
//
//
//int main()
//{
//
//
//	osgEarth::QuadTreeBuilder qtb;
//
//
//
//	float sampleRatio = 1.0f;
//	float maxError = 4.0f;
//
//	osg::ref_ptr<osg::Node> loadedModel = osgDB::readRefNodeFile("E:/DATA/dizhi/02.ive");
//	//osg::ref_ptr<osg::Node> loadedModel = osgDB::readRefNodeFile("C:/COMMON_LIBRARY/OpenSceneGraph-3.4.0--/OpenSceneGraph-Data-3.0.0/cow.osg");
//
//	if (!loadedModel)
//	{
//		return 1;
//	}
//
//	osgUtil::IndexMeshVisitor imv;
//	loadedModel->accept(imv);
//
//	unsigned int keyFlag = 0;
//
//	osgViewer::Viewer viewer;
//	viewer.addEventHandler(new KeyboardEventHandler(keyFlag));
//	viewer.setSceneData(loadedModel.get());
//	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
//	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
//	viewer.addEventHandler(new osgViewer::StatsHandler());
//	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
//	viewer.addEventHandler(new osgViewer::ThreadingHandler());
//	viewer.addEventHandler(new osgViewer::LODScaleHandler());
//	viewer.addEventHandler(new osgViewer::RecordCameraPathHandler());
//	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler());
//	viewer.realize();
//
//	float multiplier = 0.5f;
//	float minRatio = 0.00001f;
//	float ratio = sampleRatio;
//
//	while (!viewer.done())
//	{
//		viewer.frame();
//		if (keyFlag == 1 || keyFlag == 2)
//		{
//			if (keyFlag == 1) ratio *= multiplier;
//			if (keyFlag == 2) ratio /= multiplier;
//			if (ratio < minRatio) ratio = minRatio;
//
//			osgUtil::Simplifier simplifier(ratio, maxError);
//			std::cout << "Runing osgUtil::Simplifier with SampleRatio=" << ratio << " maxError=" << maxError << " ...";
//			std::cout.flush();
//			osg::ref_ptr<osg::Node> root = (osg::Node*)loadedModel->clone(osg::CopyOp::DEEP_COPY_ALL);
//			root->accept(simplifier);
//			std::cout << "done" << std::endl;
//			viewer.setSceneData(root.get());
//			keyFlag = 0;
//		}
//	}
//	return 0;
//}

#include "Cutter.h"
#include "QuadTreeBuilder.h"
#include <osg/PagedLOD>
#include <osg/ShapeDrawable>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <sstream>

class ComputeBB
{
public:
	ComputeBB::ComputeBB()
	{
		//_fileNames.clear();
		_bb.init();
	}
	void ComputeBB::traverse(const std::string& path)
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

	const osg::BoundingBox& getBB() const
	{
		return _bb;
	}

private:
	void handleFile(const std::string& filename)
	{
		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);

		osg::ComputeBoundsVisitor cbv;
		node->accept(cbv);

		osg::BoundingBox bb = cbv.getBoundingBox();

		_bb.expandBy(bb);
	}

	bool handleDir(const std::string& path)
	{
		return true;
	}

private:
	osg::BoundingBox _bb;
};


//
class ComputeTiles
{
public:
	ComputeTiles::ComputeTiles(const osg::BoundingBox& bb, const std::string& path) :_bb(bb), _resultPath(path)
	{ }

	void ComputeTiles::traverse(const std::string& path)
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
	void handleFile(const std::string& filename)
	{
		osg::ref_ptr<osg::Node> v1 = osgDB::readNodeFile(filename);

		//osg::ComputeBoundsVisitor cbv;
		//v1->accept(cbv);
		//osg::BoundingBox bb = cbv.getBoundingBox();

		std::string simplename = osgDB::getStrippedName(filename);
		//std::string resultPath = "E:/DATA/TFXQ_SW_model_V_V1.01/LOD/";

		QuadTreeBuilder qtb;
		qtb(5, v1, _bb, simplename, _resultPath);
	}

	bool handleDir(const std::string& path)
	{
		return true;
	}

private:
	std::string _resultPath;
	osg::BoundingBox _bb;
};

int main(int argc, char** argv)
{
	ComputeBB cbb;
	cbb.traverse("E:/DATA/dizhi");
	const osg::BoundingBox& bb = cbb.getBB();

	ComputeTiles ct(bb, "E:/DATA/TFXQ_SW_model_V_V1.01/LOD/");
	ct.traverse("E:/DATA/dizhi");

	//std::string filename = /*"E:/DATA/TFXQ_SW_model_V_V1.01/NEW_IVE/12_k2g.IVE"*/ "E:/DATA/dizhi/10.ive";
	//osg::ref_ptr<osg::Node> v1 = osgDB::readNodeFile(filename);

	//osg::ComputeBoundsVisitor cbv;
	//v1->accept(cbv);
	//osg::BoundingBox bb = cbv.getBoundingBox();

	//std::string simplename = osgDB::getStrippedName(filename);
	//std::string resultPath = "E:/DATA/TFXQ_SW_model_V_V1.01/LOD/";

	//QuadTreeBuilder qtb;
	//qtb(5, v1, bb, simplename, resultPath);

	system("pause");

	return 0;
}
