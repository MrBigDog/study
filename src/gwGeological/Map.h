#ifndef MAP_H
#define MAP_H 1

#include "Export.h"
#include "GeologicalLayer.h"
#include "GeologicalProfile.h"

#include <osg/Referenced>
#include <string>

class GWGEOLOGICAL_EXPORT Map :public osg::Referenced
{
public:
	struct Callback :public osg::Referenced
	{
		virtual void onGeologicalLayerAdded(GeologicalLayer0* layer) {}
		virtual void onGeologicalLayerRemoved(GeologicalLayer0* layer) {}
	};
	typedef std::vector<osg::ref_ptr<Callback> > CallbackVector;

public:
	Map();
	~Map();

	void addGeologicalLayer(GeologicalLayer0* layer);
	void removeGeologicalLayer(GeologicalLayer0* layer);
	void removeGeologicalLayer(const std::string& layerName);
	GeologicalLayer0* getGeologicalLayer(const std::string& layerName);
	void getGeologicalLayers(GeologicalLayer0Vector& out_layers);

	void addCallback(Callback* cb);
	void removeCallback(Callback* cb);

private:
	CallbackVector _callbacks;
	GeologicalLayer0Vector _geologicalLayers;
	ProfileVector _geologicalProfileLayers;
};
#endif // Map_h__
