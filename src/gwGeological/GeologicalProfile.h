#ifndef GEOLOGICALPROFILE_H
#define GEOLOGICALPROFILE_H 1

#include "Export.h"
#include "SlicePlane.h"
#include "ProfileGenerator.h"
#include "WellGenerator.h"
#include "TunnelGenerator.h"
#include "FoundationDitchGenerator.h"
#include "FenceGenerator.h"
#include <osg/Node>
#include <osg/Plane>
#include <osg/Polytope>

#include <vector>

///----------添加Customer Profile----------------------------------
class GeologicalProfileModelSource : public osg::Referenced
{
public:
	GeologicalProfileModelSource() {}
	~GeologicalProfileModelSource() {}
	osg::Node* createNode()
	{
		osg::ref_ptr<osg::Node> node = createNodeImplementation();
		return node.release();
	}
protected:
	virtual osg::Node* createNodeImplementation() = 0;

private:
	std::map<std::string, osg::ref_ptr<osg::Node> > _tileMap;
};

///----------添加已定义Profile------------------------------------
//剖切Options基类;
class BaseProfileOptions :public osg::Referenced
{
public:
	BaseProfileOptions() :_type("base"), _isOnEarth(false) {}
	const std::string& type() const { return _type; }
	const bool isOnEarth()const { return _isOnEarth; }
	virtual const SlicePLaneVector& getSlicePlanes() const { return _slicePlanes; }

protected:
	void setType(const std::string& type) { _type = type; }
	void setOnEarth(bool onEarth) { _isOnEarth = onEarth; }

private:
	bool _isOnEarth;
	std::string _type;

protected:
	SlicePLaneVector _slicePlanes;
};

//面切Options;
class PlaneProfileOptions :public BaseProfileOptions
{
public:
	PlaneProfileOptions() { setType("plane"); }
	void addPlane(const osg::Plane& plane)
	{
		_slicePlanes.push_back(SlicePlane(plane, osg::Polytope()));
	}
};

//折线切Options;
class LineStripProfileOptions : public BaseProfileOptions
{
public:
	LineStripProfileOptions() { setType("line_strip"); }
	void addLine(const osg::Vec3Array* va, bool isClosed)
	{
		ProfileStripGenerator::generateSlicePlanes(va, isClosed, isOnEarth(), _slicePlanes);
	}
};

//栅栏切Options;
class FenceProfileOptions :public BaseProfileOptions
{
public:
	FenceProfileOptions() { setType("fence"); }
	void addFence(const osg::BoundingBox& bb, const osg::Vec3d& dir, unsigned int sliceNum)
	{
		FenceGenerator::generateSlicePlanes(bb, dir, sliceNum, _slicePlanes);
	}
	void addFence(const osg::BoundingBox& bb, const osg::Vec3d& dir, double deltaDistane)
	{
		FenceGenerator::generateSlicePlanes(bb, dir, deltaDistane, _slicePlanes);
	}
};

//钻孔Options;
class WellProfileOptions : public BaseProfileOptions
{
public:
	WellProfileOptions() { setType("well"); }
	void addWell(const osg::Vec3d& point, float wellRadius, unsigned int segmentNum = 16)
	{
		WellGenerator::generateSlicePlanes(wellRadius, point, isOnEarth(), _slicePlanes, segmentNum);
	}
};

//圆形隧道Options;
class CircleTunelProfileOptions :public BaseProfileOptions
{
public:
	CircleTunelProfileOptions() { setType("circle_tunel"); }
	void addTunel(osg::Vec3dArray* va, double radius/*, double depth*/)
	{
		CircleTunelGenerator::generateSlicePlanes(va, radius, isOnEarth(), _slicePlanes);
	}

private:
	std::vector<SlicePLaneVector> _tunels;
};

//方形隧道Options;
class SquareTunelProfileOptions :public BaseProfileOptions
{
public:
	enum TopType { ARC_TOP, PLANE_TOP };

public:
	SquareTunelProfileOptions() { setType("square_tunel"); }
	void addTunel(osg::Vec3dArray* va, double width, double height, double depth, TopType topType)
	{
		SquareTunelGenerator::TopType tt = topType == ARC_TOP ? SquareTunelGenerator::ARC_TOP : SquareTunelGenerator::PLANE_TOP;
		SquareTunelGenerator::generateSlicePlanes(va, width, height, tt, isOnEarth(), _slicePlanes);
	}
};

//基坑Options;
class FoundationDitchProfileOptions :public BaseProfileOptions
{
public:
	FoundationDitchProfileOptions() { setType("doundation_ditch"); }
	void addFoundationDitch(const osg::Vec3Array* va, double depth)
	{
		FoundationDitchGenerator::generateSlicePlanes(va, isOnEarth(), _slicePlanes);
	}
};

//////////////////////////////////////////////////////////////////////////
class GWGEOLOGICAL_EXPORT LayerProfile :public osg::Referenced
{
public:
	struct Callback :public osg::Referenced
	{
		virtual void onVisibleChanged(bool isVisible) {}
	};
	typedef std::vector<osg::ref_ptr<Callback> > CallbackVector;

public:
	LayerProfile(const std::string& name, const BaseProfileOptions& options);

	const std::string& getName() const;
	const BaseProfileOptions& getOptions()const { return _options; }

	void setVisible(bool isVisible);
	bool getVisible() { return _isVisible; }

	osg::Node* getSceneGraph() { return _prifileGraph; }
	void setSceneGraph(osg::Node* sceneGraph) { _prifileGraph = sceneGraph; }

	void addCallback(Callback* cb);
	void removeCallback(Callback* cb);

private:
	bool _isVisible;
	bool _isLightEnable;
	std::string _name;
	CallbackVector _callbacks;
	osg::ref_ptr<osg::Node> _prifileGraph;
	const BaseProfileOptions _options;
};

typedef std::vector<osg::ref_ptr<LayerProfile> > ProfileVector;

#endif // GeologicalProfileLayer_h__
