#ifndef H_COOKBOOK_CH8_OCTREEBUILDER
#define H_COOKBOOK_CH8_OCTREEBUILDER

#include <osg/Geode>
#include <osg/LOD>

class OctreeBuilder
{
public:
	struct ElementInfo
	{
		ElementInfo(const std::string& name, const osg::BoundingBox& bb, osg::Node* node)
			: _name(name), _bb(bb), _node(node)
		{}
		std::string _name;
		osg::BoundingBox _bb;
		osg::ref_ptr<osg::Node> _node;
	};

public:
	OctreeBuilder() : _maxChildNumber(16), _maxTreeDepth(INT_MAX), _maxLevel(0) {}

	int getMaxLevel() const { return _maxLevel; }

	void setMaxChildNumber(int max) { _maxChildNumber = max; }
	int getMaxChildNumber() const { return _maxChildNumber; }

	void setMaxTreeDepth(int max) { _maxTreeDepth = max; }
	int getMaxTreeDepth() const { return _maxTreeDepth; }

	osg::Group* build(int depth, const osg::BoundingBox& total, const std::vector<ElementInfo>& elements);

protected:
	osg::LOD* createNewLevel(int level, const osg::Vec3& center, float radius);
	osg::Group* createBoxForDebug(const osg::Vec3& max, const osg::Vec3& min, const std::vector<ElementInfo>& elements);

	int _maxChildNumber;
	int _maxTreeDepth;
	int _maxLevel;
};


typedef std::vector<OctreeBuilder::ElementInfo> ElementInfoVec;

#endif
