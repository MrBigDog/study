#ifndef QuadTreeBuilder_h__
#define QuadTreeBuilder_h__

#include <osg/Node>
#include <osgDB/Options>

struct QuadTreeBuilder
{
public:
	QuadTreeBuilder();
	void operator()(unsigned int maxLevel, osg::Node* node, const osg::BoundingBox bbox, const std::string&name, const std::string& path);

private:
	void build(unsigned int lv, unsigned int maxL, osg::Node* original, const osg::BoundingBox bbox, const std::string&name, const std::string& path);

private:
	std::string _resultPath;
	osg::ref_ptr<osgDB::Options> _dbOptions;
};

#endif // QuadTreeBuilder_h__
