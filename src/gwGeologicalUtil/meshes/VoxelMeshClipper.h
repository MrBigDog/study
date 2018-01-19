#ifndef __MESH_H__
#define __MESH_H__

#include <osg/Geometry>

namespace gwUtil
{
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

		osg::Node* clip(osg::Geometry* geom, const osg::Vec3d& planePoint, const osg::Vec3d& planeNormal);
	private:
		FVF_TYPE _fvfType;
	};
}

#endif /* __MESH_H__ */