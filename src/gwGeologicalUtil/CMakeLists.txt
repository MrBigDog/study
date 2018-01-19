IF   (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DGWGEOLOGICALUTIL_LIBRARY)
ELSE (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DGWGEOLOGICALUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_GWEARTH)

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
ENDIF(WIN32)

SET(LIB_NAME "gwGeologicalUtil")

SET(HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME})
INCLUDE_DIRECTORIES(AFTER ${OSG_INCLUDE_DIR})

SET(LIB_PUBLIC_HEADERS
		Export.h
		Config.h
		HTTPClient.h
		IOTypes.h
		optional.h
		StringUtils.h
		ThreadingUtils.h
		URI.h
		XmlUtils.h
		Common.h
		JsonUtils.h
		FileUtils.h
		DateTime.h
		CacheBin.h
		CachePolicy.h
		Containers.h
		Registry.h
		Progress.h
		Cache.h
		tinyxml.h
		tinystr.h
		Notify.h
		ImageUtils.h
		NodeUtils.h
		MeshConsolidator.h
		PlaneIntersector.h
		LineSegmentIntersector.h
		Random.h
		#VoxelPlaneIntersector.h
		#meshes/VoxelMeshClipper.h
		SceneGraphCallback.h
)
SET(TARGET_SRC
		Config.cpp
		HTTPClient.cpp
		IOTypes.cpp
		StringUtils.cpp
		ThreadingUtils.cpp
		URI.cpp
		XmlUtils.cpp
		JsonUtils.cpp
		FileUtils.cpp
		DateTime.cpp
		CacheBin.cpp
		CachePolicy.cpp
		Registry.cpp
		Progress.cpp
		Cache.cpp
		tinyxml.cpp
		tinyxmlerror.cpp
		tinyxmlparser.cpp
		tinystr.cpp
		Notify.cpp
		ImageUtils.cpp
		NodeUtils.cpp
		MeshConsolidator.cpp
		PlaneIntersector.cpp
		LineSegmentIntersector.cpp
		Random.cpp
		#VoxelPlaneIntersector.cpp
		#VoxelMeshClipper.cpp
		#Matrix.cpp
		#Vector.cpp
		SceneGraphCallback.cpp
)

ADD_LIBRARY(
    ${LIB_NAME} ${GWEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TARGET_SRC} )
	
INCLUDE_DIRECTORIES(${GWEARTH_SOURCE_DIR}/Src ${OSG_INCLUDE_DIR} ${CURL_INCLUDE_DIR})


LINK_WITH_VARIABLES(${LIB_NAME} CURL_LIBRARY
								OSG_LIBRARY
								OSGDB_LIBRARY
								OSGGA_LIBRARY
								OSGUTIL_LIBRARY
								OSGVIEWER_LIBRARY
								OPENTHREADS_LIBRARY
								OSGTEXT_LIBRARY
								CURL_LIBRARY
								)

INCLUDE(ModuleInstall OPTIONAL)