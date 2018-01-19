#include <osg/Quat>
#include <osg/Depth>
#include <osg/TexMat>
#include <osg/CullFace>
#include <osg/Material>
#include <osg/NodeVisitor>
#include <osg/ValueObject>
#include <osg/StencilTwoSided>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>

#include <osgUtil/Optimizer>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <assert.h>
//Q32: 0
//Q31: 0
//Q4 : 0 1 2 6 7
//K2j: 1
//K2g: _index == 3 || _index == 4
//K1t-J: _index == 0

static bool isVec4Equal(const osg::Vec4& p1, const osg::Vec4& p2, float epsilon)
{
	if (!osg::equivalent(p1[0], p2[0], epsilon)) return false;
	if (!osg::equivalent(p1[1], p2[1], epsilon)) return false;
	if (!osg::equivalent(p1[2], p2[2], epsilon)) return false;
	if (!osg::equivalent(p1[3], p2[3], epsilon)) return false;
	return true;
}


class GeometryVisitor :public osg::NodeVisitor
{
public:
	GeometryVisitor(const osg::Vec4& color, osg::Image* image, bool isFault)
		: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		, _index(0)
		, _color(color)
		, _image(image)
		, _isFault(isFault)
	{
		if (_image)
		{
			_texture = new osg::Texture2D(image);
			_texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
			_texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
		}
	}

	virtual void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			//if (_index == 0 /*|| _index == 1 || _index == 2||_index==6 || _index == 7*/)
			//{
			//	osg::Vec3Array* na = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
			//	for (osg::Vec3Array::iterator it = na->begin(); it != na->end(); ++it)
			//	{
			//		*it = -(*it);
			//	}
			//}

			osg::ref_ptr <osg::Vec4Array> ca = new osg::Vec4Array;
			ca->push_back(_color);
			geom->setColorArray(ca, osg::Array::BIND_OVERALL);

			if (_texture)
			{
				geom->setUserData(_texture.get());
			}
			if (_isFault)
			{
				geom->setUserValue("isFault", true);
			}
			else
			{
				geom->setUserValue("isFault", false);
			}
			geom->setUseDisplayList(false);
			geom->setUseVertexBufferObjects(true);
		}
		_index++;
	}

private:
	osg::Vec4 _color;
	osg::ref_ptr<osg::Image> _image;
	osg::ref_ptr<osg::Texture2D> _texture;
	unsigned int _index;
	bool _isFault;
};

class ReverseNormalVisitor :public osg::NodeVisitor
{
public:
	ReverseNormalVisitor()
		: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{ }

	virtual void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			//if (_index == 0 /*|| _index == 1 || _index == 2||_index==6 || _index == 7*/)
			//{
			osg::Vec3Array* na = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
			for (osg::Vec3Array::iterator it = na->begin(); it != na->end(); ++it)
			{
				*it = -(*it);
			}

			geom->setUseDisplayList(false);
			geom->setUseVertexBufferObjects(true);
			//}
		}
	}
};

class GetGeom :public osg::NodeVisitor
{
public:
	GetGeom() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{ }

	virtual void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			////if (_index == 0 /*|| _index == 1 || _index == 2||_index==6 || _index == 7*/)
			////{
			//osg::Vec3Array* na = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
			//for (osg::Vec3Array::iterator it = na->begin(); it != na->end(); ++it)
			//{
			//	*it = -(*it);
			//}

			//geom->setUseDisplayList(false);
			//geom->setUseVertexBufferObjects(true);
			////}

			_geom = geom;
		}
	}

	osg::Geometry* getGeom()
	{
		return _geom.get();
	}

private:
	osg::ref_ptr<osg::Geometry> _geom;
};

class GeologicalDirectoryVisitor
{
public:
	GeologicalDirectoryVisitor::GeologicalDirectoryVisitor(osg::Group* root, const std::map<std::string, osg::Vec4>& cm)
		: _root(root)
		, _colorMap(cm)
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
#if 1
		if (ext != "dxf") return;

		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);
		if (!node) return;

		std::string vkey = osgDB::getNameLessAllExtensions(osgDB::getSimpleFileName(filename));
		const osg::Vec4& color = _colorMap[vkey];

		assert(!isVec4Equal(color, osg::Vec4(), FLT_EPSILON));

		bool isFault = (vkey == "fault");

		GeometryVisitor vbov(color, 0L, isFault);
		node->accept(vbov);

		std::string sname = osgDB::getNameLessExtension(filename);
		osgDB::writeNodeFile(*node, sname + ".osgb", new osgDB::Options("WriteImageHint=IncludeData Compressor=zlib"));
#else
		if (ext != "osgb") return;
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
	std::map<std::string, osg::Vec4> _colorMap;
};

int main()
{
	{
		//std::map<std::string, osg::Vec4> colorMap;
		//colorMap["1_tt"] = osg::Vec4(255 / 255.0, 170 / 255.0, 0 / 255.0, 1.0);
		//colorMap["2_Qhz"] = osg::Vec4(255 / 255.0, 255 / 255.0, 115 / 255.0, 1.0);
		//colorMap["3_Qp3g_1"] = osg::Vec4(255 / 255.0, 255 / 255.0, 0 / 255.0, 1.0);
		//colorMap["3_Qp3g_2"] = osg::Vec4(255 / 255.0, 255 / 255.0, 0 / 255.0, 1.0);
		//colorMap["4_Qp2hj_1"] = osg::Vec4(255 / 255.0, 235 / 255.0, 175 / 255.0, 1.0);
		//colorMap["4_Qp2hj_2"] = osg::Vec4(255 / 255.0, 235 / 255.0, 175 / 255.0, 1.0);
		//colorMap["4_Qp2hj_3"] = osg::Vec4(255 / 255.0, 235 / 255.0, 175 / 255.0, 1.0);
		//colorMap["5_Qp1-2m_1"] = osg::Vec4(255 / 255.0, 211 / 255.0, 127 / 255.0, 1.0);
		//colorMap["5_Qp1-2m_2"] = osg::Vec4(255 / 255.0, 211 / 255.0, 127 / 255.0, 1.0);
		//colorMap["5_Qp1-2m_3"] = osg::Vec4(255 / 255.0, 211 / 255.0, 127 / 255.0, 1.0);
		//colorMap["6_K2g_1"] = osg::Vec4(233 / 255.0, 255 / 255.0, 190 / 255.0, 1.0);
		//colorMap["6_K2g_2"] = osg::Vec4(233 / 255.0, 255 / 255.0, 190 / 255.0, 1.0);
		//colorMap["6_K2g_3"] = osg::Vec4(233 / 255.0, 255 / 255.0, 190 / 255.0, 1.0);
		//colorMap["6_K2g_4"] = osg::Vec4(233 / 255.0, 255 / 255.0, 190 / 255.0, 1.0);
		//colorMap["6_K2g_5"] = osg::Vec4(233 / 255.0, 255 / 255.0, 190 / 255.0, 1.0);
		//colorMap["7_K1-2j_1"] = osg::Vec4(209 / 255.0, 255 / 255.0, 115 / 255.0, 1.0);
		//colorMap["7_K1-2j_2"] = osg::Vec4(209 / 255.0, 255 / 255.0, 115 / 255.0, 1.0);
		//colorMap["8_K1t"] = osg::Vec4(170 / 255.0, 255 / 255.0, 0 / 255.0, 1.0);
		//colorMap["9_J3p"] = osg::Vec4(190 / 255.0, 232 / 255.0, 255 / 255.0, 1.0);
		//colorMap["10_J3sn"] = osg::Vec4(115 / 255.0, 178 / 255.0, 255 / 255.0, 1.0);
		//colorMap["11_J2s"] = osg::Vec4(115 / 255.0, 223 / 255.0, 255 / 255.0, 1.0);
		//colorMap["fault"] = osg::Vec4(255 / 255.0, 0 / 255.0, 0 / 255.0, 1.0);

		//osg::ref_ptr<osg::Group> root = new osg::Group;

		//GeologicalDirectoryVisitor gdv(root, colorMap);
		//gdv.traverse("E:\\DATA\\GeoData\\ALLmodel\\3tfsw_171010_dxf\\fault.dxf");
	}

	{
		//std::string filename = "E:/DATA/GeoData/ALLmodel/5dxq_171110_dxf/dxq_dxf_171110/4.dxf";
		//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);

		//ReverseNormalVisitor rnv;
		//node->accept(rnv);

		//std::string sname = osgDB::getNameLessExtension(filename);
		////osgDB::writeNodeFile(*node, filename);
		//osgDB::writeNodeFile(*node, sname + ".osgb"/*, new osgDB::Options("WriteImageHint=IncludeData Compressor=zlib")*/);
	}

	{
		osg::ref_ptr<osg::Node> node1 = osgDB::readNodeFile("E:/DATA/GeoData/ALLmodel/5dxq_171110_dxf/dxq_dxf_171110/2.dxf");
		osg::ref_ptr<osg::Node> node2 = osgDB::readNodeFile("E:/DATA/GeoData/ALLmodel/5dxq_171110_dxf/dxq_dxf_171110/3.dxf");

		GetGeom gg;	node1->accept(gg);
		osg::ref_ptr<osg::Geometry> geom1 = gg.getGeom();
		GetGeom gg2; node2->accept(gg2);
		osg::ref_ptr<osg::Geometry>geom2 = gg2.getGeom();

		osg::MatrixList ml1 = geom1->getWorldMatrices();
		osg::Matrixd mat1 = ml1[0];

		osg::MatrixList ml2 = geom2->getWorldMatrices();
		osg::Matrixd mat2 = ml2[0];

		osg::Matrixd mat = /*osg::Matrixd::inverse*/(mat2*osg::Matrixd::inverse(mat1));

		osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom2->getVertexArray());
		if (va)
		{
			for (osg::Vec3Array::iterator it = va->begin(); it != va->end(); ++it)
			{
				*it = (*it)*mat;
			}
		}

		osgUtil::Optimizer::MergeGeometryVisitor mgv;
		mgv.mergeGeometry(*geom1, *geom2);

		//osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		//geode->addDrawable(geom1);

		osgDB::writeNodeFile(*node1, "ts.osgb");

		//const std::vector<osg::ref_ptr<osg::PrimitiveSet> >& pl1 = geom1->getPrimitiveSetList();
		//std::vector<osg::ref_ptr<osg::PrimitiveSet> >& pl2 = geom2->getPrimitiveSetList();

		//for (unsigned int i = 0; i < pl1.size(); ++i)
		//{
		//	pl2.push_back(pl1[i]);
		//}

	}

	//root->addChild(node);

	osgViewer::Viewer viewer;
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.getDatabasePager()->setTargetMaximumNumberOfPageLOD(1);
	viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, true);
	viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	//viewer.setSceneData(root);
	viewer.setUpViewInWindow(20, 20, 1000, 800);
	viewer.realize();

	while (!viewer.done())
	{
		viewer.frame();
	}
	viewer.run();

	return 0;
}