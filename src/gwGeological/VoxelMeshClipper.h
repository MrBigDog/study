#ifndef __MESH_H__
#define __MESH_H__

#include <osg/Geometry>


//----------------------------------------------------------------------------------------------------
class VolelMeshClipper
{
public:
	enum FVF_TYPE
	{
		VT,  //vertex texcoord
		VNC, //vertex normal color
		VNT, //vertex normal texcoord
		VNTC //vertex normal color texcoord
	};

public:
	VolelMeshClipper(FVF_TYPE fvfType = VNC);
	~VolelMeshClipper();

	static osg::Node* clipGeom(osg::Geometry* geom, const osg::Vec3d& planePoint, const osg::Vec3d& planeNormal);
	static osg::Node* clipGeom(osg::Geometry* geom, const osg::Plane& plane);
	static osg::Node* clipNode(osg::Node* node, const osg::Plane& plane);

private:
	FVF_TYPE _fvfType;
};


#endif /* __MESH_H__ */