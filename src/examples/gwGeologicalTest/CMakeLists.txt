
SET(TARGET_H adjust_near_far_plane.hpp)
SET(TARGET_SRC  main.cpp )

INCLUDE_DIRECTORIES(${GWEARTH_SOURCE_DIR}/Src ${OSGEARTH_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

SET(TARGET_LIBRARIES_VARS 
	OSG_LIBRARY 
	OSGGA_LIBRARY 
	OSGDB_LIBRARY 
	OSGUTIL_LIBRARY 
	OSGVIEWER_LIBRARY 
	OPENTHREADS_LIBRARY
	)
	
SET(TARGET_COMMON_LIBRARIES gwGeological gwGeologicalUtil )
	
SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_LIBDIR})

SETUP_APPLICATION(gwGeologicalTest)