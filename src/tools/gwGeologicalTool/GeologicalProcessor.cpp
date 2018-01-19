#include "GeologicalProcessor.h"

#include <gwGeologicalUtil/Notify.h>
#include <gwGeologicalUtil/XmlUtils.h>

#include <osg/Texture2D>
#include <osg/NodeVisitor>
#include <osg/ValueObject>
#include <osg/PagedLOD>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgUtil/Simplifier>

#define UID "uid"
#define NORMAL_MAP "normal_map"
#define ATTRIBUATE "attribuate"
#define DIFFUSE_MAP "diffuse_map"
#define LOCATION_XYZ "location_xyz" //bb center
#define GEOLOGICAL_MODEL "geological_model"
#define MODEL_CALASS "model_class"

namespace
{
	class DirectoryVisitor
	{
	public:
		DirectoryVisitor::DirectoryVisitor()
		{
			_fileNames.clear();
		}
		void DirectoryVisitor::traverse(const std::string& path)
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

		const StringVector& getFileNameList() { return _fileNames; }

	private:
		void handleFile(const std::string& filename)
		{
			_fileNames.push_back(filename);
		}

		bool handleDir(const std::string& path)
		{
			return true;
		}

	private:
		StringVector _fileNames;
	};

	class ColorSetter :public osg::NodeVisitor
	{
	public:
		ColorSetter(const osg::Vec4& color)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _color(color)
		{}

		void apply(osg::Geode& geode)
		{
			for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
			{
				osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
				if (!geom) continue;

				osg::Vec4Array* ca = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
				if (!ca) continue;

				for (osg::Vec4Array::iterator it = ca->begin(); it != ca->end(); ++it)
				{
					(*it) = _color;
				}
			}
		}
	private:
		osg::Vec4 _color;
	};

	bool saveConfigToXml(const gwUtil::Config& config, const std::string& outXmlPath)
	{
		osg::ref_ptr<gwUtil::XmlDocument> doc = new gwUtil::XmlDocument(config);

		std::string path = osgDB::getFilePath(outXmlPath);
		if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
		{
			GW_WARN << "GeologicalModelTool" << "Couldn't create path " << std::endl;
		}
		std::ofstream out(outXmlPath.c_str());
		doc->store(out);

		return true;
	}
}

//----------------------------------------------------------------------------------------------------
GeologicalProcessor::GeologicalProcessor()
{}

void GeologicalProcessor::traverse(const std::string & path, StringVector& fileNameList)
{
	DirectoryVisitor dv;
	dv.traverse(path);
	fileNameList = dv.getFileNameList();
}

void GeologicalProcessor::setCurrentNode(const std::string & nodePath)
{
	_currentNode = osgDB::readNodeFile(nodePath);
}

void GeologicalProcessor::setDiffuseMap(const std::string & diffuseMapName)
{
	if (!_currentNode) return;
	osg::ref_ptr < osg::Image > image = osgDB::readImageFile(diffuseMapName);
	if (!image) return;

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(image);
	_currentNode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
}

void GeologicalProcessor::split(double sizeFactor)
{

}

osg::Node * GeologicalProcessor::getCurrentNode()
{
	return _currentNode;
}

void GeologicalProcessor::publish(const std::string & path)
{
	//写模型和配置文件;
	saveConfigToXml(_config, osgDB::concatPaths(path, "GeologicalModel.xml"));
}

void GeologicalProcessor::setColor(const osg::Vec4 & color)
{
	if (!_currentNode) return;

	ColorSetter cs(color);
	_currentNode->accept(cs);
}

//void GeologicalProcessor::setNormalMap(osg::Node* node, const std::string & normalMapName)
//{
//}

void GeologicalProcessor::setUid(const std::string & uid)
{
	_currentNode->setUserValue("uid", uid);
}

void GeologicalProcessor::setAttribuate(const std::string & attr)
{
	_currentNode->setUserValue("attribute", attr);
}
