#ifndef GEOLOGICALLAYER_H
#define GEOLOGICALLAYER_H 1

#include "Export.h"
#include <map>
#include <string>
#include <osg/Node>
#include <osg/observer_ptr>
#include "GeologicalProfile.h"

//----------自定义地质体模型Source
class GeologicalModelSource : public osg::Referenced
{
public:
	GeologicalModelSource() {}
	~GeologicalModelSource() {}
	osg::Node* createNode()
	{
		osg::ref_ptr<osg::Node> node = createNodeImplementation();
		return node.release();
	}
private:
	virtual osg::Node* createNodeImplementation() = 0;
};


typedef std::map<LayerProfile*, osg::ref_ptr<osg::Group> > ProfileNodeMap;

class GWGEOLOGICAL_EXPORT GeologicalLayer0 :public osg::Referenced
{
public:
	struct Callback :public osg::Referenced
	{
		virtual void onVisibleChanged(bool isVisible) {}
		virtual void onLightEnableChanged(bool isLightEnable) {}

		virtual void onProfileAdded(LayerProfile* profile) {}
		virtual void onProfileRemoved(LayerProfile* profile) {}
	};
	typedef std::vector<osg::ref_ptr<Callback> > CallbackVector;

public:
	GeologicalLayer0(const std::string& name, osg::Node* node);
	GeologicalLayer0(const std::string& name, const std::string& url);
	GeologicalLayer0(const std::string& name, GeologicalModelSource* modelSource);

	~GeologicalLayer0();

	const std::string& getName() const;

	void addProfile(LayerProfile* profile);
	void removeProfile(LayerProfile* profile);
	LayerProfile* getProfile(const std::string& profileName) const;
	osg::Node* getProfileNode(LayerProfile* profile) const;
	osg::Group* getProfileRoot() const { return _profileRoot.get(); }
	void setProfileRoot(osg::Group* node) { _profileRoot = node; }
	void getProfiles(ProfileVector& profiles) const;
	const ProfileNodeMap& getProfileNodeMap() const;
	ProfileNodeMap& getProfileNodeMap();

	osg::Node* getSceneGraph() const { return _graph.get(); }
	osg::Node* getOrCreateSceneGraph();

	void setVisible(bool isVisible);
	bool getVisible() const;

	void setLightEnable(bool isLightEnable);
	bool getLightEnable() const;

	void addCallback(Callback* cb);
	void removeCallback(Callback* cb);

private:
	bool _isVisible;
	bool _isLightEnable;

	std::string _name;
	ProfileVector _profileVec;
	ProfileNodeMap _profileNodeMap;
	CallbackVector _callbacks;
	osg::observer_ptr<osg::Node> _graph;
	osg::ref_ptr<osg::Group> _profileRoot;
	osg::ref_ptr<GeologicalModelSource> _modelSource;
};

typedef std::vector<osg::ref_ptr<GeologicalLayer0> > GeologicalLayer0Vector;

#endif // GeologicalLayer_h__
