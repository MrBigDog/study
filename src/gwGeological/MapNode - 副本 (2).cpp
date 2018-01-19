#include "MapNode.h"
#include <gwGeologicalUtil/StringUtils.h>
#include <gwGeologicalUtil/NodeUtils.h>
#include <gwGeologicalUtil/SceneGraphCallback.h>
#include <osg/Geode>
#include <osg/PagedLOD>
#include <osg/observer_ptr>
#include <osg/ValueObject>
#include <osg/PolygonMode>
#include <osgUtil/CullVisitor>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <sstream>
#include <iostream>

using namespace gwUtil;

namespace
{
	class PagedLodVisitor :public osg::NodeVisitor
	{
	public:
		PagedLodVisitor(/*SceneGraphCallbacks* cbs*/)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _readoptions(0L)
			, _callbacks(0L)
		{}

		void setOptions(osgDB::Options* options) { _readoptions = options; }
		void setSceneGraphCallbacks(gwUtil::SceneGraphCallbacks* cbs) { _callbacks = cbs; }

	private:
		void apply(osg::PagedLOD& pl)
		{
			osg::ref_ptr<PagedLODWithSceneGraphCallbacks> newpl = new PagedLODWithSceneGraphCallbacks(pl);
			if (_callbacks.valid())
			{
				newpl->setSceneGraphCallbacks(_callbacks);
			}
			if (_readoptions.valid())
			{
				newpl->setDatabaseOptions(_readoptions);
			}
			osg::Group* parent = findFirstParentOfType<osg::Group>(&pl);
			if (parent)
			{
				parent->replaceChild(&pl, newpl);
			}
		}
	private:
		osg::ref_ptr<osgDB::Options> _readoptions;
		osg::ref_ptr<SceneGraphCallbacks> _callbacks;
	};

	class MySceneGraphCallback :public gwUtil::SceneGraphCallback
	{
	public:
		MySceneGraphCallback(/*osg::Group* group*/) /*:_root(group)*/
		{}

		void onPostMergeNode(osg::Node* node)
		{
			osg::ref_ptr<SceneGraphCallbacks> cbs = new SceneGraphCallbacks;
			cbs->add(new MySceneGraphCallback());

			PagedLodVisitor plv;
			plv.setSceneGraphCallbacks(cbs);
			node->accept(plv);

			osg::ref_ptr<osg::Node> profileNode;
			_root->addChild(profileNode);
		}

		void onRemoveNode(osg::Node* node) { }

	private:
		osg::ref_ptr<osg::Group> _root;
	};

	//class GeologicalModelReaderWriter :public osgDB::ReaderWriter
	//{
	//public:
	//	GeologicalModelReaderWriter() { }
	//	virtual bool acceptsExtension(const std::string& extension) const
	//	{
	//		return osgDB::equalCaseInsensitive(extension, "geological_tile");
	//	}

	//	virtual ReadResult readNode(const std::string& uri, const Options* options) const
	//	{
	//		std::string ext = osgDB::getLowerCaseFileExtension(uri);
	//		if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;

	//		std::string realUri = osgDB::getNameLessExtension(uri);
	//		std::string realExt = osgDB::getLowerCaseFileExtension(realUri);

	//		osg::ref_ptr<osgDB::ReaderWriter> rw;
	//		rw = osgDB::Registry::instance()->getReaderWriterForExtension(realExt);
	//		if (!rw) return ReadResult::FILE_NOT_HANDLED;

	//		osgDB::ReaderWriter::ReadResult readresult = rw->readNode(realUri, options);
	//		if (!readresult.success())
	//		{
	//			return ReadResult::FILE_NOT_HANDLED;
	//		}
	//		osg::Node* resultnode = readresult.takeNode();

	//		PagedLodVisitor plv;
	//		osg::ref_ptr<gwUtil::SceneGraphCallbacks> cbs = new gwUtil::SceneGraphCallbacks;
	//		cbs->add(new MySceneGraphCallback(0));
	//		plv.setSceneGraphCallbacks(cbs);
	//		resultnode->accept(plv);

	//		//在这里添加profile;
	//		if (resultnode)
	//		{
	//			PagedLodVisitor plv;
	//			resultnode->accept(plv);
	//		}
	//		return resultnode;
	//	}
	//};
	//REGISTER_OSGPLUGIN(geological_tile, GeologicalModelReaderWriter)

		//
	class GeologicalLayerNode :public osg::Group
	{
	private:
		std::string _name;
		osg::ref_ptr<GeologicalLayer0> _layer;
		osg::ref_ptr<osg::Group> _profileRoot;
		osg::ref_ptr<osg::Group> _geologicalRoot;

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
		osg::Node* getProfileRoot() const { return _profileRoot.get(); }
		osg::Node* getGeologicalRoot() const { return _geologicalRoot.get(); }
		void addGeologicalNode(osg::Node*node) { _geologicalRoot->addChild(node); }

		void onProfileAdded(LayerProfile* profile)
		{
			if (!profile) return;

			osg::ref_ptr<osg::Node> profileNode = _layer->getProfileNodeMap()[profile];
			if (profileNode.valid()) return;

			//create profile node
			osg::ref_ptr<osg::Group> rr = new osg::Group;
			traverseNode(_geologicalRoot, rr);

			_profileRoot->addChild(rr);
			_layer->getProfileNodeMap()[profile] = rr;

			if (profileNode.valid())
			{
				_profileRoot->addChild(profileNode);
				_layer->getProfileNodeMap()[profile] = profileNode;
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
		void traverseNode(osg::Node* node, osg::Group* group)
		{
			if (!node) return;

			osg::PagedLOD* pl = dynamic_cast<osg::PagedLOD*>(node);
			if (pl)
			{
				osg::ref_ptr<osg::PagedLOD> newPl = new osg::PagedLOD;
				group->addChild(newPl);
				for (unsigned int i = 0; i < pl->getNumChildren(); ++i)
				{
					osg::Node* child = pl->getChild(i);
					traverseNode(node, newPl);
				}
			}
			else
			{
				osg::Geode* geode = node->asGeode();
				if (geode)
				{
					osg::ref_ptr<osg::Node> profileNode = 0L;
					if (profileNode.valid())
					{
						osg::PagedLOD* pl = gwUtil::findFirstParentOfType<osg::PagedLOD>(geode);
						if (pl) profileNode->setUserValue("file_name", pl->getFileName(1));
						group->addChild(profileNode);
					}
				}
				else
				{
					osg::Group* g = node->asGroup();
					if (g)
					{
						osg::ref_ptr<osg::Group> newgroup = new osg::Group;
						group->addChild(newgroup);
						for (unsigned int i = 0; i < g->getNumChildren(); ++i)
						{
							osg::Node* child = g->getChild(i);
							traverseNode(node, newgroup);
						}
					}
					else
					{
						osg::ref_ptr<osg::Node> profileNode = 0L;
						osg::PagedLOD* pl = gwUtil::findFirstParentOfType<osg::PagedLOD>(geode);
						if (pl) profileNode->setUserValue("file_name", pl->getFileName(1));
						if (profileNode.valid())
						{
							group->addChild(profileNode);
						}
					}
				}
			}
		}
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