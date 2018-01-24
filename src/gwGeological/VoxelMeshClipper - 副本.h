#ifndef VOXELMESHCLIPPER_H
#define VOXELMESHCLIPPER_H 1

#include <osg/Node>
#include <osg/Geometry>

class VolelMeshClipper
{
public:
	enum FVF_TYPE
	{
		VT,  //vertex, texcoord
		VNC, //vertex, normal, color
		VNT, //vertex, normal, texcoord
		VNTC //vertex, normal, color, texcoord
	};

public:
	VolelMeshClipper(FVF_TYPE fvfType = VNC);
	~VolelMeshClipper();

	static osg::Node* clipGeom(osg::Geometry* geom, const osg::Plane& plane);
	static osg::Node* clipNode(osg::Node* node, const osg::Plane& plane);

private:
	FVF_TYPE _fvfType;
};


#endif /* VOXELMESHCLIPPER_H */