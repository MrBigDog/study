#include "MapNode.h"
#include "VoxelMeshClipper.h"
#include "GeologicalProfile.h"
#include <gwGeologicalUtil/NodeUtils.h>
#include <gwGeologicalUtil/StringUtils.h>
#include <gwGeologicalUtil/SceneGraphCallback.h>

#include <osg/Geode>
#include <osg/PagedLOD>
#include <osg/ValueObject>
#include <osg/PolygonMode>
#include <osg/observer_ptr>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgUtil/CullVisitor>

#include <sstream>
#include <iostream>

#define SUBLOD_FLAG "issublod"

using namespace gwUtil;

namespace
{
	static osg::Node* generateProfileNode(osg::Node* geoNode, LayerProfile* profile)
	{
		osg::ref_ptr<osg::Group> root = 0L;
		const SlicePLaneVector& planes = profile->getOptions().getSlicePlanes();
		for (SlicePLaneVector::const_iterator pit = planes.begin(); pit != planes.end(); ++pit)
		{
			osg::ref_ptr<osg::Node> pnode = VolelMeshClipper::clipNode(geoNode, (*pit)._plane);
			if (pnode.valid())
			{
				if (!root.valid()) root = new osg::Group;
				root->addChild(pnode);
			}
		}
		return root.release();
	}

	class MyLod :public osg::LOD
	{
	public:
		void setAttatchNode(osg::Node* node) { _attatchNode = node; }
		osg::Node* getAttatchNode() const { return _attatchNode; }
	private:
		osg::Node* _attatchNode;
	};

	struct MyLodVisitor : public osg::NodeVisitor
	{
		MyLodVisitor(osg::Node*attatchNode)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _attatchNode(attatchNode), _myLod(0L) { }

		void apply(osg::Node& node)
		{
			if (!_attatchNode) return;
			MyLod* result = dynamic_cast<MyLod*>(&node);
			if (result)
			{
				if (result->getAttatchNode() == _attatchNode)
				{
					_myLod = result;
					return;
				}
			}
			traverse(node);
		}
		MyLod* getResult() const { return _myLod; }
	private:
		osg::Node* _attatchNode;
		MyLod* _myLod;
	};

	static MyLod* findMyLod(osg::Node* attatchNode, osg::Node* root)
	{
		if (!attatchNode || !root) return 0L;
		MyLodVisitor fmlv(attatchNode);
		root->accept(fmlv);
		return fmlv.getResult();
	}

	class ProfileGenerator :public gwUtil::BFSNodeVisitor
	{
	public:
		ProfileGenerator(LayerProfile* profile) :_profile(profile), _result(0L) { }
		osg::Node* getResult() const { return _result.get(); }

	private:
		void traverseBFS(osg::Node& node)
		{
			bool issublod = false;
			if (node.getUserValue(SUBLOD_FLAG, issublod) && issublod == true)
			{
				return;
			}
			gwUtil::BFSNodeVisitor::traverseBFS(node);
		}

		void apply(osg::Geode& geode)
		{
			if (!_profile) return;

			osg::ref_ptr<osg::Node> profilenode = generateProfileNode(&geode, _profile);
			if (profilenode.valid())
			{
				if (!_result) _result = new osg::Group;
				_result->addChild(profilenode);
			}
		}

	private:
		LayerProfile* _profile;
		osg::ref_ptr<osg::Group> _result;
	};

	class GeologicalLayerNode :public osg::Group
	{
	private:
		osg::ref_ptr<osg::Group> _profileRoot;
		osg::ref_ptr<osg::Group> _geologicalRoot;
		osg::ref_ptr<GeologicalLayer0> _layer;

		class UpdateProfileCallback :public GeologicalLayer0::Callback
		{
		public:
			UpdateProfileCallback(GeologicalLayerNode* layerNode) :_layerNode(layerNode) {}

			virtual void onProfileAdded(LayerProfile* profile)
			{
				if (!_layerNode.valid()) return;
				_layerNode->onProfileAdded(profile);
			}
			virtual void onProfileRemoved(LayerProfile* profile)
			{
				if (!_layerNode.valid()) return;
				_layerNode->onProfileRemoved(profile);
			}
		private:
			osg::ref_ptr<GeologicalLayerNode> _layerNode;
		};

	public:
		GeologicalLayerNode(GeologicalLayer0* layer) :_layer(0L)
		{
			if (layer)
			{
				_layer = layer;
				_profileRoot = new osg::Group;
				_geologicalRoot = new osg::Group;
				this->addChild(_profileRoot);
				this->addChild(_geologicalRoot);

				layer->addCallback(new UpdateProfileCallback(this));
				layer->setProfileRoot(_profileRoot);
				setName(layer->getName() + "_NODE");

				ProfileVector profiles;
				layer->getProfiles(profiles);
				for (ProfileVector::const_iterator it = profiles.begin(); it != profiles.end(); ++it)
				{
					onProfileAdded(*it);
				}
				profiles.clear();
			}
		}

		GeologicalLayer0* getLayer() const { return _layer.get(); }
		osg::Group* getProfileRoot() const { return _profileRoot.get(); }

		void addGeologicalNode(osg::Node*node)
		{
			_geologicalRoot->addChild(node);
		}

		void onProfileAdded(LayerProfile* profile)
		{
			if (!profile) return;

			osg::ref_ptr<MyLod> profileNode = 0L;
			traverseNode(profile, _geologicalRoot, profileNode, _geologicalRoot);

			if (profileNode.valid())
			{
				_profileRoot->addChild(profileNode);
				_layer->getProfileNodeMap()[profile] = profileNode;
				profile->setSceneGraph(profileNode);
			}
		}

		void onProfileRemoved(LayerProfile* profile)
		{
			ProfileNodeMap::iterator it = _layer->getProfileNodeMap().find(profile);
			if (it != _layer->getProfileNodeMap().end())
			{
				_profileRoot->removeChild(it->second);
				_layer->getProfileNodeMap().erase(it);
			}
		}

	private:
		void traverseNode(LayerProfile* profile, osg::Node* node, MyLod* parentMylod, osg::Group* parentGeoModel)
		{
			if (!node) return;
			if (!parentMylod) parentMylod = new MyLod;

			bool isSublod = false;
			if (!node->getUserValue(SUBLOD_FLAG, isSublod))
			{
				osg::Group* g = dynamic_cast<osg::Group*>(node);
				if (g)
				{
					for (unsigned int i = 0; i < g->getNumChildren(); ++i)
					{
						traverseNode(profile, g->getChild(i), parentMylod, parentGeoModel);
					}
				}
			}
			else
			{
				if (isSublod == 1)
				{
					osg::ref_ptr<MyLod> mylod = new MyLod;
					mylod->setAttatchNode(node);

					ProfileGenerator pg(profile);
					node->accept(pg);
					osg::ref_ptr<osg::Node> cpnode = pg.getResult();
					if (cpnode.valid())
					{
						mylod->addChild(cpnode);
					}

					PagedLODWithSceneGraphCallbacks* parent = findFirstParentOfType<PagedLODWithSceneGraphCallbacks>(node);
					if (parent && parentGeoModel->containsNode(parent))
					{
						unsigned int ind = parent->getChildIndex(node);
						if (ind < parent->getNumRanges())
						{
							const std::pair<float, float>& range = parent->getRangeList()[ind];
							parentMylod->addChild(mylod, range.first, range.second);
						}
					}
					else
					{
						parentMylod->addChild(mylod);
					}
					osg::Group* g = dynamic_cast<osg::Group*>(node);
					if (g)
					{
						for (unsigned int i = 0; i < g->getNumChildren(); ++i)
						{
							traverseNode(profile, g->getChild(i), mylod, dynamic_cast<osg::Group*>(node));
						}
					}
				}
				else
				{
					osg::Group* g = dynamic_cast<osg::Group*>(node);
					if (g)
					{
						for (unsigned int i = 0; i < g->getNumChildren(); ++i)
						{
							traverseNode(profile, g->getChild(i), parentMylod, parentGeoModel);
						}
					}
				}
			}
		}
	};

	class PagedLodVisitor :public osg::NodeVisitor
	{
	public:
		PagedLodVisitor(SceneGraphCallbacks* cbs, osgDB::Options* options = 0L)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _readoptions(options)
			, _callbacks(cbs)
		{}

	private:
		void apply(osg::PagedLOD& pl)
		{
			osg::ref_ptr<PagedLODWithSceneGraphCallbacks> newpl = new PagedLODWithSceneGraphCallbacks(pl);
			if (_callbacks.valid()) newpl->setSceneGraphCallbacks(_callbacks);
			if (_readoptions.valid()) newpl->setDatabaseOptions(_readoptions);
			osg::Group* parent = findFirstParentOfType<osg::Group>(&pl);
			if (parent) parent->replaceChild(&pl, newpl);//此处会不会冲突？--不会，因为是在pagedlod thread中执行的;
		}
	private:
		osg::ref_ptr<osgDB::Options> _readoptions;
		osg::ref_ptr<SceneGraphCallbacks> _callbacks;
	};

	//实现对ProfileNode（剖面）的调度;---鼓掌
	class GenerateProfileNodeCallback :public gwUtil::SceneGraphCallback
	{
	public:
		GenerateProfileNodeCallback(GeologicalLayer0* layer, osg::Group* geoParent)
			: _layer(layer)
			, _geoParent(geoParent)
		{}

		void onPostMergeNode(osg::Node* node)//在这里实现剖面的添加;
		{
			node->setUserValue(SUBLOD_FLAG, true);
			osg::ref_ptr<SceneGraphCallbacks> cbs = new SceneGraphCallbacks;
			cbs->add(new GenerateProfileNodeCallback(_layer, dynamic_cast<osg::Group*>(node)));
			PagedLodVisitor plv(cbs);
			node->accept(plv);

			ProfileVector profiles;
			_layer->getProfiles(profiles);

			if (!profiles.empty())
			{
				for (ProfileVector::const_iterator it = profiles.begin(); it != profiles.end(); ++it)
				{
					osg::Group* profileRoot = _layer->getProfileNodeMap()[(*it)];
					if (!profileRoot)//初始化出问题了;
					{
						continue;
					}

					osg::ref_ptr<MyLod> currentProfileLod = findMyLod(node, profileRoot);//
					if (currentProfileLod)
					{
						continue;//这是一种邪门的情况;
					}

					osg::ref_ptr<osg::Node> profileNode = generateProfileNode(node, *it);
					if (!profileNode.valid())
					{
						continue;
					}

					currentProfileLod = new MyLod;
					currentProfileLod->setAttatchNode(node);
					currentProfileLod->addChild(profileNode);

					MyLod* parentProfileLod = findMyLod(_geoParent, profileRoot);
					if (!parentProfileLod)
					{
						continue;//error;
					}

					PagedLODWithSceneGraphCallbacks* parent = findFirstParentOfType<PagedLODWithSceneGraphCallbacks>(node);
					if (parent && _geoParent->containsNode(parent))
					{
						int rangeIndex = parent->getNumChildren() - 1;
						if (rangeIndex < (int)parent->getRangeList().size())
						{
							const std::pair<float, float>& range = parent->getRangeList()[rangeIndex];
							parentProfileLod->addChild(currentProfileLod, range.first, range.second);
						}
						else
						{
							parentProfileLod->addChild(currentProfileLod);
						}
					}
					else
					{
						parentProfileLod->addChild(currentProfileLod);
					}
				}
			}
		}

		void onRemoveNode(osg::Node* node)//在这里实现对剖面的删除;
		{
			if (!_layer) return;

			ProfileVector profiles;
			_layer->getProfiles(profiles);
			if (profiles.empty()) return;

			for (ProfileVector::const_iterator it = profiles.begin(); it != profiles.end(); ++it)
			{
				MyLod* myLod = findMyLod(node, _layer->getProfileNodeMap()[*it]);
				if (!myLod) continue;

				osg::Group* parent = findFirstParentOfType<osg::Group>(myLod);
				if (!parent) continue;

				parent->removeChild(myLod);
			}
		}

	private:
		osg::Group* _geoParent;
		osg::ref_ptr<GeologicalLayer0> _layer;
	};

	//--------------------------------------------------------------------------------------------------------
	struct UpdateMapCallback :public Map::Callback
	{
		UpdateMapCallback(MapNode* node) :_node(node) {}
		virtual void onGeologicalLayerAdded(GeologicalLayer0* layer)
		{
			_node->onGeologicalLayerAdded(layer);
		}
		virtual void onGeologicalLayerRemoved(GeologicalLayer0* layer)
		{
			_node->onGeologicalLayerRemoved(layer);
		}
	private:
		osg::observer_ptr<MapNode> _node;
	};
}

//////////////////////////////////////////////////////////////////////////
MapNode::MapNode(Map * map) :_map(map)
{
	//地质体模型根节点;
	_geologicalRoot = new Group;
	_geologicalRoot->setName("GEOLOGICAL_MODEL_ROOT");
	this->addChild(_geologicalRoot);

	GeologicalLayer0Vector geoLayers;
	_map->getGeologicalLayers(geoLayers);
	for (GeologicalLayer0Vector::const_iterator it = geoLayers.begin(); it != geoLayers.end(); ++it)
	{
		onGeologicalLayerAdded(*it);
	}
	geoLayers.clear();
	_map->addCallback(new UpdateMapCallback(this));
}

MapNode::~MapNode() { }

void MapNode::onGeologicalLayerAdded(GeologicalLayer0 * layer)
{
	if (!layer) return;

	osg::ref_ptr<GeologicalLayerNode> layernode = dynamic_cast<GeologicalLayerNode*>
		(_geologicalLayerNodeMap[layer].get());
	if (layernode.valid()) return;

	osg::Node* node = layer->getOrCreateSceneGraph();
	if (!node) return;

	layernode = new  GeologicalLayerNode(layer);
	layernode->addGeologicalNode(node);

	osg::ref_ptr<SceneGraphCallbacks> cbs = new SceneGraphCallbacks();
	cbs->add(new GenerateProfileNodeCallback(layer, NULL));
	PagedLodVisitor plv(cbs);
	node->accept(plv);

	_geologicalRoot->addChild(layernode);
	_geologicalLayerNodeMap[layer] = layernode;
}

void MapNode::onGeologicalLayerRemoved(GeologicalLayer0 * layer)
{
	GeologicalLayerNodeMap::iterator it = _geologicalLayerNodeMap.find(layer);
	if (it == _geologicalLayerNodeMap.end()) return;

	_geologicalRoot->removeChild(it->second);
	_geologicalLayerNodeMap.erase(it);
}

osg::Node * MapNode::getGeologicalLayerNode(GeologicalLayer0 * layer)
{
	GeologicalLayerNodeMap::const_iterator it = _geologicalLayerNodeMap.find(layer);
	if (it == _geologicalLayerNodeMap.end()) return 0L;
	return it->second;
}