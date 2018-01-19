IF   (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DGWGEOLOGICAL_LIBRARY)
ELSE (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DGWGEOLOGICAL_LIBRARY_STATIC)
ENDIF(DYNAMIC_GWEARTH)

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
ENDIF(WIN32)

include("${CGAL_DIR}/UseCGAL.cmake")#CGALÏà¹Ø¿â

SET(LIB_NAME "gwGeological")

SET(HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME})
INCLUDE_DIRECTORIES(AFTER ${OSG_INCLUDE_DIR})

SET(LIB_PUBLIC_HEADERS
		Export.h
		Cutter.h
		AssistPlaneGenerator.h
		FenceGenerator.h
		FoundationDitchGenerator.h
		GeneratorHelp.h
		GeologicalLayer.h
		GeologicalProfile.h
		#LineSegmentIntersector.h
		Map.h
		MapNode.h
		#MeshConsolidator.h
		#NodeUtils.h
		#PlaneIntersector.h
		ProfileGenerator.h
		SliceOperator.h
		SlicePlane.h
		#TileRegister.h
		TunnelGenerator.h
		WellGenerator.h
		clipper.hpp
		VoxelMeshClipper.h
		Vector.h
		#TrenchGenerator.h
)
SET(TARGET_SRC
		AssistPlaneGenerator.cpp
		FenceGenerator.cpp
		FoundationDitchGenerator.cpp
		GeneratorHelp.cpp
		GeologicalLayer.cpp
		GeologicalProfile.cpp
		#LineSegmentIntersector.cpp
		Map.cpp
		MapNode.cpp
		#MeshConsolidator.cpp
		#NodeUtils.cpp
		#PlaneIntersector.cpp
		ProfileGenerator.cpp
		SliceOperator.cpp
		SlicePlane.cpp
		#TileRegister.cpp
		TunnelGenerator.cpp
		WellGenerator.cpp
		clipper.cpp
		VoxelMeshClipper.cpp
		Vector.cpp
		#TrenchGenerator.cpp
)

ADD_LIBRARY(
    ${LIB_NAME} ${GWEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TARGET_SRC} )
	
INCLUDE_DIRECTORIES(${GWEARTH_SOURCE_DIR}/Src ${OSG_INCLUDE_DIR} ${Boost_INCLUDE_DIR})


LINK_WITH_VARIABLES(${LIB_NAME} CURL_LIBRARY
								OSG_LIBRARY
								OSGDB_LIBRARY
								OSGGA_LIBRARY
								OSGUTIL_LIBRARY
								OSGVIEWER_LIBRARY
								OPENTHREADS_LIBRARY
								OSGTEXT_LIBRARY
								)
								
LINK_INTERNAL(${LIB_NAME} gwGeologicalUtil)

INCLUDE(ModuleInstall OPTIONAL)

