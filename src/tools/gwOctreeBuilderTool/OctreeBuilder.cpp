#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/ComputeBoundsVisitor>
#include "OctreeBuilder.h"

osg::Group* OctreeBuilder::build(int depth, const osg::BoundingBox& total, const std::vector<ElementInfo>& elements)
{
	int s[3];  // axis sides (0 or 1)
	osg::Vec3 extentSet[3] =
	{
		total._min,
		(total._max + total._min) * 0.5f,
		total._max
	};

	std::vector<ElementInfo> childData;
	for (unsigned int i = 0; i < elements.size(); ++i)
	{
		const ElementInfo& obj = elements[i];
		if (total.contains(obj._bb._min) && total.contains(obj._bb._max))
		{
			childData.push_back(obj);
		}
		else if (total.intersects(obj._bb))
		{
			osg::Vec3 center = (obj._bb._max + obj._bb._min) * 0.5f;
			if (total.contains(center))
			{
				childData.push_back(obj);
			}
		}
	}

	bool isLeafNode = false;
	if ((int)childData.size() <= _maxChildNumber || depth > _maxTreeDepth)
	{
		isLeafNode = true;
	}

	osg::ref_ptr<osg::Group> group = new osg::Group;
	if (!isLeafNode)
	{
		osg::ref_ptr<osg::Group> childNodes[8];
		for (s[0] = 0; s[0] < 2; ++s[0])
		{
			for (s[1] = 0; s[1] < 2; ++s[1])
			{
				for (s[2] = 0; s[2] < 2; ++s[2])
				{
					// Calculate the child extent
					osg::Vec3 min, max;
					for (int a = 0; a < 3; ++a)
					{
						min[a] = (extentSet[s[a] + 0])[a];
						max[a] = (extentSet[s[a] + 1])[a];
					}

					int id = s[0] + (2 * s[1]) + (4 * s[2]);
					childNodes[id] = build(depth + 1, osg::BoundingBox(min, max), childData);
				}
			}
		}

		for (unsigned int i = 0; i < 8; ++i)
		{
			if (childNodes[i] && childNodes[i]->getNumChildren())
				group->addChild(childNodes[i]);
		}
	}
	else
	{
		for (unsigned int i = 0; i < childData.size(); ++i)
		{
			const ElementInfo& obj = childData[i];
			osg::Vec3 center = (obj._bb._max + obj._bb._min) * 0.5;
			float radius = (obj._bb._max - obj._bb._min).length() * 0.5f;
			group->addChild(obj._node);
		}
	}

	osg::Vec3 center = (total._max + total._min) * 0.5;
	float radius = (total._max - total._min).length() * 0.5f;

	osg::LOD* level = createNewLevel(depth, center, radius);

	level->insertChild(0, createBoxForDebug(total._max, total._min, childData));
	level->insertChild(1, group.get());
	return level;
}

osg::LOD* OctreeBuilder::createNewLevel(int level, const osg::Vec3& center, float radius)
{
	float range = radius*30.0f;
	osg::ref_ptr<osg::LOD> lod = new osg::LOD;
	lod->setCenterMode(osg::LOD::USER_DEFINED_CENTER);
	lod->setCenter(center);
	lod->setRadius(radius);
	lod->setRange(0, range, FLT_MAX);
	lod->setRange(1, 0.0f, range);

	if (_maxLevel < level) _maxLevel = level;
	return lod.release();
}

static osg::Geode* createDebugBox(const osg::Vec3& max, const osg::Vec3& min, const osg::Vec4& color = osg::Vec4(1, 1, 1, 1))
{
	osg::Vec3 dir = max - min;
	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array(10);
	(*va)[0] = min + osg::Vec3(0.0f, 0.0f, 0.0f);
	(*va)[1] = min + osg::Vec3(0.0f, 0.0f, dir[2]);
	(*va)[2] = min + osg::Vec3(dir[0], 0.0f, 0.0f);
	(*va)[3] = min + osg::Vec3(dir[0], 0.0f, dir[2]);
	(*va)[4] = min + osg::Vec3(dir[0], dir[1], 0.0f);
	(*va)[5] = min + osg::Vec3(dir[0], dir[1], dir[2]);
	(*va)[6] = min + osg::Vec3(0.0f, dir[1], 0.0f);
	(*va)[7] = min + osg::Vec3(0.0f, dir[1], dir[2]);
	(*va)[8] = min + osg::Vec3(0.0f, 0.0f, 0.0f);
	(*va)[9] = min + osg::Vec3(0.0f, 0.0f, dir[2]);

	osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
	ca->push_back(color);

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(va.get());
	geom->setColorArray(ca, osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 10));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.get());
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	return geode.release();
}

osg::Geode* createBox(const osg::Vec3& max, const osg::Vec3& min, const osg::Vec4& color = osg::Vec4(1, 1, 1, 1))
{
	osg::Vec3 dir = max - min;
	//osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array(24);
	//osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array(1);
	//osg::ref_ptr<osg::Vec3Array> na = new osg::Vec3Array(24);

	//(*va)[0] = min + osg::Vec3(0.0f, 0.0f, 0.0f);      (*na)[0] = osg::Vec3(0.0f, -1.0f, 0.0f);
	//(*va)[1] = min + osg::Vec3(0.0f, 0.0f, dir[2]);    (*na)[1] = osg::Vec3(0.0f, -1.0f, 0.0f);
	//(*va)[2] = min + osg::Vec3(dir[0], 0.0f, 0.0f);    (*na)[2] = osg::Vec3(0.0f, -1.0f, 0.0f);
	//(*va)[3] = min + osg::Vec3(dir[0], 0.0f, dir[2]);  (*na)[3] = osg::Vec3(0.0f, -1.0f, 0.0f);

	//(*va)[4] = min + osg::Vec3(dir[0], 0.0f, 0.0f);     (*na)[4] = osg::Vec3(1.0f, 0.0f, 0.0f);
	//(*va)[5] = min + osg::Vec3(dir[0], 0.0f, dir[2]);   (*na)[5] = osg::Vec3(1.0f, 0.0f, 0.0f);
	//(*va)[6] = min + osg::Vec3(dir[0], dir[1], 0.0f);   (*na)[6] = osg::Vec3(1.0f, 0.0f, 0.0f);
	//(*va)[7] = min + osg::Vec3(dir[0], dir[1], dir[2]); (*na)[7] = osg::Vec3(1.0f, 0.0f, 0.0f);

	//(*va)[8] = min + osg::Vec3(dir[0], dir[1], 0.0f);   (*na)[8] = osg::Vec3(0.0f, 1.0f, 0.0f);
	//(*va)[9] = min + osg::Vec3(dir[0], dir[1], dir[2]); (*na)[9] = osg::Vec3(0.0f, 1.0f, 0.0f);
	//(*va)[10] = min + osg::Vec3(0.0f, dir[1], 0.0f);    (*na)[10] = osg::Vec3(0.0f, 1.0f, 0.0f);
	//(*va)[11] = min + osg::Vec3(0.0f, dir[1], dir[2]);  (*na)[11] = osg::Vec3(0.0f, 1.0f, 0.0f);

	//(*va)[12] = min + osg::Vec3(0.0f, dir[1], 0.0f);    (*na)[12] = osg::Vec3(-1.0f, 0.0f, 0.0f);
	//(*va)[13] = min + osg::Vec3(0.0f, dir[1], dir[2]);  (*na)[13] = osg::Vec3(-1.0f, 0.0f, 0.0f);
	//(*va)[14] = min + osg::Vec3(0.0f, 0.0f, 0.0f);      (*na)[14] = osg::Vec3(-1.0f, 0.0f, 0.0f);
	//(*va)[15] = min + osg::Vec3(0.0f, 0.0f, dir[2]);    (*na)[15] = osg::Vec3(-1.0f, 0.0f, 0.0f);

	//(*va)[16] = min + osg::Vec3(0.0f, 0.0f, 0.0f);      (*na)[16] = osg::Vec3(0.0f, 0.0f, -1.0f);
	//(*va)[17] = min + osg::Vec3(dir[0], 0.0f, 0.0f);    (*na)[17] = osg::Vec3(0.0f, 0.0f, -1.0f);
	//(*va)[18] = min + osg::Vec3(0.0f, dir[1], 0.0f);    (*na)[18] = osg::Vec3(0.0f, 0.0f, -1.0f);
	//(*va)[19] = min + osg::Vec3(dir[0], dir[1], 0.0f);  (*na)[19] = osg::Vec3(0.0f, 0.0f, -1.0f);

	//(*va)[20] = min + osg::Vec3(0.0f, 0.0f, dir[2]);      (*na)[20] = osg::Vec3(0.0f, 0.0f, 1.0f);
	//(*va)[21] = min + osg::Vec3(dir[0], 0.0f, dir[2]);    (*na)[21] = osg::Vec3(0.0f, 0.0f, 1.0f);
	//(*va)[22] = min + osg::Vec3(0.0f, dir[1], dir[2]);    (*na)[22] = osg::Vec3(0.0f, 0.0f, 1.0f);
	//(*va)[23] = min + osg::Vec3(dir[0], dir[1], dir[2]);  (*na)[23] = osg::Vec3(0.0f, 0.0f, 1.0f);

	//ca->push_back(color);

	//osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	//geom->setVertexArray(va.get());
	//geom->setNormalArray(na.get(), osg::Array::BIND_PER_VERTEX);
	//geom->setColorArray(ca, osg::Array::BIND_OVERALL);
	//geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 24));

	osg::ref_ptr<osg::Box> box = new osg::Box((max + min)*0.5, dir[0], dir[1], dir[2]);
	osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(box);
	sd->setColor(color);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(sd);

	return geode.release();
}

osg::Group* OctreeBuilder::createBoxForDebug(const osg::Vec3& max, const osg::Vec3& min, const std::vector<ElementInfo>& elements)
{
	osg::ref_ptr<osg::Node> debugBox = createDebugBox(max, min, osg::Vec4(1, 1, 0, 1));
	debugBox->getOrCreateStateSet()->setAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));

	osg::ref_ptr<osg::Node> nodedata = 0L;
	if (!elements.empty())
	{
		osg::ref_ptr<osg::Group> elementGroup = new osg::Group;
		for (ElementInfoVec::const_iterator it = elements.begin(); it != elements.end(); ++it)
		{
			elementGroup->addChild((*it)._node);
		}
		osg::ComputeBoundsVisitor cbv;
		elementGroup->accept(cbv);
		const osg::BoundingBox& bb = cbv.getBoundingBox();
		nodedata = createBox(bb._max, bb._min, osg::Vec4(1, 1, 1, 1));
	}

	osg::ref_ptr<osg::Group> result = new osg::Group;
	result->addChild(debugBox);
	if (nodedata.valid())
	{
		result->addChild(nodedata);
	}
	return result.release();
}
