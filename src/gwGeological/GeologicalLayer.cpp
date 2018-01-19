#include "GeologicalLayer.h"
#include <gwGeologicalUtil/NodeUtils.h>
#include <osg/PagedLOD>
#include <osg/ValueObject>
#include <osgDB/Options>
#include <osgDB/ReadFile>

#include <deque>

namespace
{
	struct NodeModelSource : public GeologicalModelSource
	{
		NodeModelSource(osg::Node* node) : _node(node) { }
		osg::Node* createNodeImplementation() { return _node.get(); }
	private:
		osg::ref_ptr<osg::Node> _node;
	};

	struct UrlModelSource :public GeologicalModelSource
	{
		UrlModelSource(const std::string& url) :_url(url) {}
		osg::Node* createNodeImplementation() { return osgDB::readNodeFile(_url); }
	private:
		std::string _url;
	};
}

//////////////////////////////////////////////////////////////////////////
GeologicalLayer0::GeologicalLayer0(const std::string& name, const std::string& url)
	: _name(name)
	, _modelSource(new UrlModelSource(url))
	, _graph(0L)
	, _profileRoot(0L)
{}

GeologicalLayer0::GeologicalLayer0(const std::string & name, GeologicalModelSource * modelSource)
	: _name(name)
	, _modelSource(modelSource)
	, _graph(0L)
	, _profileRoot(0L)
{ }

GeologicalLayer0::GeologicalLayer0(const std::string & name, osg::Node * node)
	: _name(name)
	, _modelSource(new NodeModelSource(node))
	, _graph(0L)
	, _profileRoot(0L)
{ }

GeologicalLayer0::~GeologicalLayer0() {}

const std::string & GeologicalLayer0::getName() const
{
	return _name;
}

void GeologicalLayer0::addProfile(LayerProfile* profile)
{
	if (!profile) return;
	_profileVec.push_back(profile);
	for (CallbackVector::const_iterator it = _callbacks.begin(); it != _callbacks.end(); ++it)
	{
		(*it)->onProfileAdded(profile);
	}
}

void GeologicalLayer0::removeProfile(LayerProfile* profile)
{
	ProfileVector::iterator it = std::find(_profileVec.begin(), _profileVec.end(), profile);
	if (it == _profileVec.end()) return;

	_profileVec.erase(it);
	for (CallbackVector::const_iterator it = _callbacks.begin(); it != _callbacks.end(); ++it)
	{
		(*it)->onProfileRemoved(profile);
	}
}

LayerProfile * GeologicalLayer0::getProfile(const std::string & profileName) const
{
	for (ProfileVector::const_iterator it = _profileVec.begin(); it != _profileVec.end(); ++it)
	{
		if ((*it)->getName() == profileName) return *it;
	}
	return 0L;
}

osg::Node * GeologicalLayer0::getProfileNode(LayerProfile * profile) const
{
	ProfileNodeMap::const_iterator it = _profileNodeMap.find(profile);
	if (it == _profileNodeMap.end())
	{
		return 0L;
	}
	return it->second.get();
}

void GeologicalLayer0::getProfiles(ProfileVector & profiles) const
{
	for (ProfileVector::const_iterator it = _profileVec.begin(); it != _profileVec.end(); ++it)
	{
		profiles.push_back(*it);
	}
}

const ProfileNodeMap & GeologicalLayer0::getProfileNodeMap() const
{
	return _profileNodeMap;
}

ProfileNodeMap & GeologicalLayer0::getProfileNodeMap()
{
	return _profileNodeMap;
}

void GeologicalLayer0::setVisible(bool isVisible)
{
	if (!_graph.valid()) return;

	if (_isVisible == isVisible) return;
	_isVisible = isVisible;

	_graph->setNodeMask(isVisible ? ~0 : 0);
	for (CallbackVector::const_iterator it = _callbacks.begin(); it != _callbacks.end(); ++it)
	{
		(*it)->onVisibleChanged(isVisible);
	}
}

bool GeologicalLayer0::getVisible() const
{
	return _isVisible;
}

void GeologicalLayer0::setLightEnable(bool isLightEnable)
{
	if (!_graph) return;

	if (_isLightEnable == isLightEnable) return;
	_isLightEnable = isLightEnable;
	_graph->getOrCreateStateSet()->setMode(GL_LIGHTING, _isLightEnable ? osg::StateAttribute::ON : osg::StateAttribute::OFF);
	for (CallbackVector::const_iterator it = _callbacks.begin(); it != _callbacks.end(); ++it)
	{
		(*it)->onLightEnableChanged(isLightEnable);
	}
}

bool GeologicalLayer0::getLightEnable() const
{
	return _isLightEnable;
}

void GeologicalLayer0::addCallback(Callback * cb)
{
	if (!cb) return;
	_callbacks.push_back(cb);
}

void GeologicalLayer0::removeCallback(Callback * cb)
{
	if (!cb) return;
	CallbackVector::iterator it = std::find(_callbacks.begin(), _callbacks.end(), cb);
	if (it == _callbacks.end()) return;
	_callbacks.erase(it);
}

osg::Node * GeologicalLayer0::getOrCreateSceneGraph()
{
	if (!_graph.valid())
	{
		if (_modelSource) _graph = _modelSource->createNode();
	}
	return _graph.get();
}
