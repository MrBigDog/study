#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgUtil/Simplifier>
#include <osgUtil/SmoothingVisitor>

class ColorVisitor :public osg::NodeVisitor
{
public:
	ColorVisitor(const osg::Vec4& color)
		:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		, _color(color)
	{}

	virtual void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			osg::Vec4 color = _color;

			osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
			ca->push_back(color);

			geom->setColorArray(ca, osg::Vec4Array::BIND_OVERALL);

			geom->dirtyDisplayList();
			geom->dirtyBound();
		}
		geode.dirtyBound();
	}
private:
	osg::Vec4 _color;
};

class GeologicalDirectoryVisitor
{
public:
	GeologicalDirectoryVisitor::GeologicalDirectoryVisitor()
	{ }

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
		std::string ext = osgDB::getFileExtension(filename);
		if (ext != "dxf") return;

		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);
		if (!node) return;

		std::string simplename = osgDB::getSimpleFileName(filename);

		osg::Vec4 color(1, 0, 0, 1);
		if (simplename == "1_Q4.dxf")
		{
			color = osg::Vec4(228 / 255.0, 255 / 255.0, 204 / 255.0, 1.0);
		}
		else if (simplename == "2_Q4.dxf")
		{
			color = osg::Vec4(231 / 255.0, 256 / 255.0, 231 / 255.0, 1.0);
		}
		else if (simplename == "3_Q4.dxf")
		{
			color = osg::Vec4(255 / 255.0, 255 / 255.0, 216 / 255.0, 1.0);
		}
		else if (simplename == "4_Q32.dxf")
		{
			color = osg::Vec4(255 / 255.0, 239 / 255.0, 203 / 255.0, 1.0);
		}
		else if (simplename == "5_Q32.dxf")
		{
			color = osg::Vec4(255 / 255.0, 247 / 255.0, 230 / 255.0, 1.0);
		}
		else if (simplename == "6_Q31.dxf")
		{
			color = osg::Vec4(255 / 255.0, 210 / 255.0, 165 / 255.0, 1.0);
		}
		else if (simplename == "7_K1t-J.dxf")
		{
			color = osg::Vec4(255 / 255.0, 216 / 255.0, 191 / 255.0, 1.0);
		}
		else if (simplename == "8_K2j.dxf")
		{
			color = osg::Vec4(255 / 255.0, 215 / 255.0, 239 / 255.0, 1.0);
		}
		else if (simplename == "9_K2j.dxf")
		{
			color = osg::Vec4(255 / 255.0, 231 / 255.0, 255 / 255.0, 1.0);
		}
		else if (simplename == "10_K2g.dxf")
		{
			color = osg::Vec4(231 / 255.0, 199 / 255.0, 183 / 255.0, 1.0);
		}
		else if (simplename == "11_K2g.dxf")
		{
			color = osg::Vec4(239 / 255.0, 208 / 255.0, 200 / 255.0, 1.0);
		}
		else if (simplename == "12_k2g.dxf")
		{
			color = osg::Vec4(247 / 255.0, 232 / 255.0, 232 / 255.0, 1.0);
		}
		else if (simplename == "13_J3S.dxf")
		{
			color = osg::Vec4(255 / 255.0, 191 / 255.0, 191 / 255.0, 1.0);
		}
		else
		{
			color = osg::Vec4(255 / 255.0, 0 / 255.0, 0 / 255.0, 1.0);
		}
		ColorVisitor gv(color);
		node->accept(gv);

		//osgUtil::SmoothingVisitor sv;
		//node->accept(sv);

		//osgUtil::Simplifier simplifier(0.3, 4.0);
		//node->accept(simplifier);

		std::string sname = osgDB::getNameLessExtension(filename);
		osgDB::writeNodeFile(*node, sname + ".IVE");
	}

	bool GeologicalDirectoryVisitor::handleDir(const std::string& path)
	{
		return true;
	}
};


class DirectoryVisitor
{
public:
	DirectoryVisitor() {}

	virtual void handleFile(const std::string& filename)
	{
		filenames.push_back(filename);
	}

	virtual bool handleDir(const std::string& path) { return true; }

	virtual void traverse(const std::string& path)
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


	void writeToLocal(const std::string& resName)
	{
		std::ofstream out(resName/*"filename_list.txt"*/);
		for (std::vector<std::string>::const_iterator it = filenames.begin(); it != filenames.end(); ++it)
		{
			out << *it << std::endl;
		}
	}

private:
	std::vector< std::string > filenames;
};

int main()
{
	//GeologicalDirectoryVisitor gdv;
	//gdv.traverse("E:/DATA/tfsw_17.9.21");
	DirectoryVisitor dv;
	dv.traverse("E:\\gwGeological\\GWGeoV2\\model_plg\\src\\interface");
	dv.writeToLocal("filename_list.txt");

	return 0;
}