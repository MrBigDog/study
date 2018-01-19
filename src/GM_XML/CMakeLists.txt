IF   (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DGM_XML_LIBRARY)
ELSE (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DGM_XML_LIBRARY_STATIC)
ENDIF(DYNAMIC_GWEARTH)

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
	LIST(APPEND TARGET_EXTERNAL_LIBRARIES psapi)
ENDIF(WIN32)

SET(LIB_NAME "GM_XML")

SET(HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME})
#INCLUDE_DIRECTORIES(AFTER ${OSG_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(AFTER ${OSGEARTH_INCLUDE_DIR})

SET(LIB_PUBLIC_HEADERS
	Export.h
	AbstractFeature.h
	AbstractGeometry.h
	AbstractGML.h
	AbstractSpatialReferenceSystem.h
	AbstractTexture.h
	CoverageStyle.h
	FeatureClass.h
	FeatureRelation.h
	FeatureStyle.h
	Geo3DLayer.h
	Geo3DMap.h
	Geo3DMapMetaData.h
	Geo3DProject.h
	Geo3DProjectMetaData.h
	Geo3DStyle.h
	GeologicFeature.h
	#GeologicFeatureRelation.h
	GeologicRelation.h
	GeoModel.h
	GeoModelMetaData.h
	GeoSceneStyle.h
	GeoSymbolizer.h
	gmmlDisplayErrorandWarning.h
	gmmlSystem.h
	gmmlSystemInfo.h
	MappedFeature.h
	SpatialRelation.h
)

SET(TARGET_SRC
	AbstractFeature.cpp
	AbstractGeometry.cpp
	AbstractGML.cpp
	AbstractSpatialReferenceSystem.cpp
	AbstractTexture.cpp
	CoverageStyle.cpp
	FeatureClass.cpp
	FeatureRelation.cpp
	FeatureStyle.cpp
	Geo3DLayer.cpp
	Geo3DMap.cpp
	Geo3DMapMetaData.cpp
	Geo3DProject.cpp
	Geo3DProjectMetaData.cpp
	Geo3DStyle.cpp
	GeologicFeature.cpp
	#GeologicFeatureRelation.cpp
	GeologicRelation.cpp
	GeoModel.cpp
	GeoModelMetaData.cpp
	GeoSceneStyle.cpp
	GeoSymbolizer.cpp
	gmmlDisplayErrorandWarning.cpp
	gmmlSystemInfo.cpp
	MappedFeature.cpp
	SpatialRelation.cpp
)

ADD_LIBRARY(
    ${LIB_NAME} ${GWEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TARGET_SRC} )
	
INCLUDE_DIRECTORIES(${GWEARTH_SOURCE_DIR}/Src ${GWEARTH_SOURCE_DIR}/Src/GM_XML )#

#IF (WIN32)
#  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
#ELSE(WIN32)
#  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
#ENDIF(WIN32)

#OPTION(NRL_STATIC_LIBRARIES "Link osgEarth against static GDAL and cURL, including static OpenSSL, Proj4, JPEG, PNG, and TIFF." OFF)
#if(NOT NRL_STATIC_LIBRARIES)
#  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY ZLIB_LIBRARIES)
#else(NOT NRL_STATIC_LIBRARIES)
#  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY #OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY ZLIB_LIBRARIES OSGMANIPULATOR_LIBRARY SSL_EAY_RELEASE LIB_EAY_RELEASE TIFF_LIBRARY PROJ4_LIBRARY PNG_LIBRARY JPEG_LIBRARY)
#endif(NOT NRL_STATIC_LIBRARIES)

#LINK_WITH_VARIABLES(${LIB_NAME} 
#								OSG_LIBRARY 
#								OSGVIEWER_LIBRARY
#								OPENTHREADS_LIBRARY
#								OSGEARTH_LIBRARY
#								ZLIB_LIBRARIES
#								)
 
#LINK_INTERNAL(${LIB_NAME} shlwapi)
#LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)