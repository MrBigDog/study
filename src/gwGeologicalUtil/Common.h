#ifndef GWGEOLOGICALUTIL_COMMON_H
#define GWGEOLOGICALUTIL_COMMON_H 1

#include "Export.h"
#include <osg/Version>
#include <osg/ref_ptr>
#include <osg/Referenced>

// define the OSG Version checks for older OSG versions

#ifndef OSG_MIN_VERSION_REQUIRED
#  define OSG_MIN_VERSION_REQUIRED(MAJOR, MINOR, PATCH) ((OPENSCENEGRAPH_MAJOR_VERSION>MAJOR) || (OPENSCENEGRAPH_MAJOR_VERSION==MAJOR && (OPENSCENEGRAPH_MINOR_VERSION>MINOR || (OPENSCENEGRAPH_MINOR_VERSION==MINOR && OPENSCENEGRAPH_PATCH_VERSION>=PATCH))))
#  define OSG_VERSION_LESS_THAN(MAJOR, MINOR, PATCH) ((OPENSCENEGRAPH_MAJOR_VERSION<MAJOR) || (OPENSCENEGRAPH_MAJOR_VERSION==MAJOR && (OPENSCENEGRAPH_MINOR_VERSION<MINOR || (OPENSCENEGRAPH_MINOR_VERSION==MINOR && OPENSCENEGRAPH_PATCH_VERSION<PATCH))))
#  define OSG_VERSION_LESS_OR_EQUAL(MAJOR, MINOR, PATCH) ((OPENSCENEGRAPH_MAJOR_VERSION<MAJOR) || (OPENSCENEGRAPH_MAJOR_VERSION==MAJOR && (OPENSCENEGRAPH_MINOR_VERSION<MINOR || (OPENSCENEGRAPH_MINOR_VERSION==MINOR && OPENSCENEGRAPH_PATCH_VERSION<=PATCH))))
#  define OSG_VERSION_GREATER_THAN(MAJOR, MINOR, PATCH) ((OPENSCENEGRAPH_MAJOR_VERSION>MAJOR) || (OPENSCENEGRAPH_MAJOR_VERSION==MAJOR && (OPENSCENEGRAPH_MINOR_VERSION>MINOR || (OPENSCENEGRAPH_MINOR_VERSION==MINOR && OPENSCENEGRAPH_PATCH_VERSION>PATCH))))
#  define OSG_VERSION_GREATER_OR_EQUAL(MAJOR, MINOR, PATCH) ((OPENSCENEGRAPH_MAJOR_VERSION>MAJOR) || (OPENSCENEGRAPH_MAJOR_VERSION==MAJOR && (OPENSCENEGRAPH_MINOR_VERSION>MINOR || (OPENSCENEGRAPH_MINOR_VERSION==MINOR && OPENSCENEGRAPH_PATCH_VERSION>=PATCH))))
#endif

// text inlining macro
#define GW_MULTILINE(...) #__VA_ARGS__

namespace gwUtil
{
	// application-wide unique ID.
	typedef int UID;

	// reference counted unique UID.
	struct GWGEOLOGICALUTIL_EXPORT RefUID : public osg::Referenced
	{
		RefUID() : _uid(0) { }
		RefUID(UID uid) : _uid(uid) { }
		operator UID() const { return _uid; }
		UID _uid;
	};
}

#ifdef _MSC_VER
// VS ignores
#pragma warning (disable: 4224)
#pragma warning (disable: 4180)
#endif

#endif // GWGEOLOGICALUTIL_COMMON_H
