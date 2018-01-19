#ifndef GWGEOLOGICALUTIL_MESHCONSOLIDATOR_H
#define GWGEOLOGICALUTIL_MESHCONSOLIDATOR_H 1

#include "Export.h"
#include <osg/Geode>
#include <osg/Geometry>

namespace gwUtil
{
	class GWGEOLOGICALUTIL_EXPORT MeshConsolidator
	{
	public:
		static void convertToTriangles(osg::Geometry& geom, bool force = false);
		static void run(osg::Geode& geode);
	};
}

#endif // GWGEOLOGICALUTIL_MESHCONSOLIDATOR_H
