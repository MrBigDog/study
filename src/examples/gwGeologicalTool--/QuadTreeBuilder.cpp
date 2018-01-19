#include "QuadTreeBuilder.h"
#include "Cutter.h"
#include <osg/Polytope>
#include <osg/PagedLOD>
#include <osg/ValueObject>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgUtil/Simplifier>
#include <sstream>

namespace
{
	std::string makeUrl(const std::string&name, int l, int x, int y)
	{
		std::stringstream ss;
		ss << name << "_" << l << "_" << x << "_" << y;
		return ss.str();
	}

	std::string makeKey(/*const std::string&name,*/ int l, int x, int y)
	{
		std::stringstream ss;
		ss << l << "_" << x << "_" << y;
		return ss.str();
	}

	osg::PagedLOD* createPagedLod(osg::Node* node, const osg::Vec3d& center, double radius, const std::string& filename)
	{
		osg::ref_ptr<osg::PagedLOD> plod = new osg::PagedLOD;
		plod->setCenterMode(osg::PagedLOD::USER_DEFINED_CENTER);
		plod->setCenter(center);
		plod->setRadius(radius);

		float cutoff = radius * 2.0f;
		plod->insertChild(0, node);
		plod->setRange(0, cutoff, FLT_MAX);

		plod->setRange(1, 0.0f, cutoff);
		plod->setFileName(1, filename + ".OSGB");

		return plod.release();
	}
	typedef std::map<std::string, osg::ref_ptr<osg::Node> >TileMap;

	void getSubTiles(osg::Node* original, double xmin, double ymin, double maxExtend, unsigned int level, const std::string&name, const std::string&path, TileMap& tiles)
	{
		if (!original) return;
		if (level == 0)
		{
			std::string key = makeUrl(name, 0, 0, 0);
			tiles[key] = original;
			return;
		}

		unsigned int tileNum = 1 << level;
		unsigned int cutNum = tileNum - 1;
		double delta = maxExtend / (double)tileNum;

		//计算y方向的切刀;
		std::vector<osg::Plane> yPlanes;
		for (unsigned int y = 1; y < tileNum; ++y)
		{
			osg::Vec3d pos(xmin, ymin + delta*y, 0.0);
			osg::Plane yplane(osg::Vec3d(0, 1, 0), pos);
			yPlanes.push_back(yplane);
		}

		//x direction
		std::vector<osg::ref_ptr<osg::Node> > xResult;
		osg::ref_ptr<osg::Node> node = original;
		for (unsigned int x = 1; x < tileNum; ++x)
		{
			if (!node) break;
			osg::Vec3d pos = osg::Vec3d(xmin + delta*x, ymin, 0.0);
			osg::Plane xplane(osg::Vec3d(1, 0, 0), pos);

			Cutter xcutter(xplane);
			node->accept(xcutter);

			osg::ref_ptr<osg::Node> negativeNode = xcutter.getNegative();
			xResult.push_back(negativeNode);

			osg::ref_ptr<osg::Node> positiveNode = xcutter.getPositive();
			if (x == tileNum - 1)
			{
				xResult.push_back(positiveNode);
			}
			else
			{
				node = positiveNode;
			}
		}

		// y direction
		for (unsigned int x = 0; x < xResult.size(); ++x)
		{
			for (unsigned int y = 0; y < cutNum; ++y)
			{
				if (!xResult[x]) break;

				Cutter ycutter(yPlanes[y]);
				xResult[x]->accept(ycutter);

				osg::ref_ptr<osg::Node> subNegative = ycutter.getNegative();
				if (subNegative)
				{
					std::string key = makeUrl(name, level, x, y);
					tiles[key] = subNegative;
					std::string respath = osgDB::concatPaths(path, key + ".OSGB");
					//osgDB::writeNodeFile(*subNegative, respath);
				}
				osg::ref_ptr<osg::Node> subPositive = ycutter.getPositive();
				if (subPositive)
				{
					std::string key = makeUrl(name, level, x, y + 1);
					if (y == cutNum - 1)
					{
						tiles[key] = subPositive;
						std::string respath = osgDB::concatPaths(path, key + ".OSGB");
						//osgDB::writeNodeFile(*subPositive, respath);
					}
					else
					{
						xResult[x] = subPositive;
					}
				}
			}
		}
	}
}

//-------------------------------------------------------------------------------------------------------------
QuadTreeBuilder::QuadTreeBuilder()
{
	_dbOptions = new osgDB::Options(/*"WriteImageHint=WriteOut"*/);
}

void QuadTreeBuilder::operator()(unsigned int maxLevel, osg::Node* node, const osg::BoundingBox bbox, const std::string&name, const std::string& path)
{
	if (!node) return;
	for (unsigned int i = 0; i < maxLevel; ++i)
	{
		osg::ref_ptr<osg::Node> copynode = osg::clone(node, osg::CopyOp::DEEP_COPY_ALL);
		if (!copynode) continue;

		float simpleRatio = (float)(i + 1) / ((float)maxLevel + 1.0);
		osgUtil::Simplifier sf(simpleRatio, 10.0f);
		copynode->accept(sf);

		build(i, maxLevel - 1, copynode, bbox, name, path);
	}
}

void QuadTreeBuilder::build(unsigned int lv, unsigned int maxL, osg::Node* original, const osg::BoundingBox bbox, const std::string&name, const std::string& path)
{
	TileMap tiles;
	osg::ref_ptr<osg::Node> node = original;

	double maxExtend = osg::maximum(bbox.xMax() - bbox.xMin(), bbox.yMax() - bbox.yMin());

	getSubTiles(node, bbox.xMin(), bbox.yMin(), maxExtend, lv, name, path, tiles);
	if (tiles.empty()) return;

	bool hasPagedLod = lv != maxL;
	if (lv == 0)
	{
		TileMap::const_iterator it = tiles.begin();
		osg::ref_ptr<osg::Node> node = tiles[makeUrl(name, 0, 0, 0)];
		if (!node.valid())
		{
			return;
		}
		node->setUserValue("tile_key", makeUrl(name, 0, 0, 0)/* makeKey(0, 0, 0)*/);
		node->setUserValue("L", 0);
		node->setUserValue("X", 0);
		node->setUserValue("Y", 0);

		std::string filepath = osgDB::concatPaths(path, makeUrl(name, lv, 0, 0) + ".OSGB");
		if (hasPagedLod)
		{
			osg::ref_ptr<osg::PagedLOD> plod = createPagedLod(it->second, bbox.center(), bbox.radius(), makeUrl(name, 1, 0, 0));
			osgDB::writeNodeFile(*plod, filepath, _dbOptions);
		}
		else
		{
			osgDB::writeNodeFile(*node, filepath, _dbOptions);
		}
	}
	else
	{
		unsigned int tilenum = 1 << lv;
		unsigned int tileNumHalf = tilenum*0.5;
		double delta = maxExtend / (double)tileNumHalf;

		for (unsigned int x = 0; x < tileNumHalf; ++x)
		{
			unsigned int x0 = 2 * x;
			unsigned int x1 = 2 * x + 1;
			double px0 = bbox.xMin() + delta*x;
			double px1 = bbox.xMin() + delta*(x + 1);
			double pxc = (px0 + px1)*0.5;
			for (unsigned int y = 0; y < tileNumHalf; ++y)
			{
				unsigned int y0 = 2 * y;
				unsigned int y1 = 2 * y + 1;
				double py0 = bbox.yMin() + delta*y;
				double py1 = bbox.yMin() + delta*(y + 1);
				double pyc = (py0 + py1)*0.5;

				std::string key0 = makeUrl(name, lv, x0, y0);
				std::string key1 = makeUrl(name, lv, x0, y1);
				std::string key2 = makeUrl(name, lv, x1, y0);
				std::string key3 = makeUrl(name, lv, x1, y1);

				osg::ref_ptr<osg::Group> root = new osg::Group;
				osg::ref_ptr<osg::Node> node0 = tiles[key0];
				if (node0.valid())
				{
					node0->setUserValue("tile_key", key0/*makeKey(lv, x0, y0)*/);
					node0->setUserValue("L", lv);
					node0->setUserValue("X", x0);
					node0->setUserValue("Y", y0);
					osg::BoundingBox bb(px0, py0, bbox.zMin(), pxc, pyc, bbox.zMax());
					if (hasPagedLod)
						root->addChild(createPagedLod(node0, bb.center(), bb.radius(), makeUrl(name, lv + 1, x0, y0)));
					else
						root->addChild(node0);
				}
				osg::ref_ptr<osg::Node> node1 = tiles[key1];
				if (node1.valid())
				{
					node1->setUserValue("tile_key", key1/*makeKey(lv, x0, y1)*/);
					node1->setUserValue("L", lv);
					node1->setUserValue("X", x0);
					node1->setUserValue("Y", y1);
					osg::BoundingBox bb(px0, pyc, bbox.zMin(), pxc, py1, bbox.zMax());
					if (hasPagedLod)
						root->addChild(createPagedLod(node1, bb.center(), bb.radius(), makeUrl(name, lv + 1, x0, y1)));
					else
						root->addChild(node1);
				}
				osg::ref_ptr<osg::Node> node2 = tiles[key2];
				if (node2.valid())
				{
					node2->setUserValue("tile_key", key2/*makeKey(lv, x1, y0)*/);
					node2->setUserValue("L", lv);
					node2->setUserValue("X", x1);
					node2->setUserValue("Y", y0);
					osg::BoundingBox bb(pxc, py0, bbox.zMin(), px1, pyc, bbox.zMax());
					if (hasPagedLod)
						root->addChild(createPagedLod(node2, bb.center(), bb.radius(), makeUrl(name, lv + 1, x1, y0)));
					else
						root->addChild(node2);
				}
				osg::ref_ptr<osg::Node> node3 = tiles[key3];
				if (node3.valid())
				{
					node3->setUserValue("tile_key", key3/*makeKey(lv, x1, y1)*/);
					node3->setUserValue("L", lv);
					node3->setUserValue("X", x1);
					node3->setUserValue("Y", y1);
					osg::BoundingBox bb(pxc, pyc, bbox.zMin(), px1, py1, bbox.zMax());
					if (hasPagedLod)
						root->addChild(createPagedLod(node3, bb.center(), bb.radius(), makeUrl(name, lv + 1, x1, y1)));
					else
						root->addChild(node3);
				}
				if (root->getNumChildren() > 0)
				{
					std::string filepath = osgDB::concatPaths(path, makeUrl(name, lv, x, y) + ".OSGB");
					osgDB::writeNodeFile(*root, filepath, _dbOptions);
				}
			}
		}
	}
}
