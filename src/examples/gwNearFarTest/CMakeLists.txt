SET(TARGET_H)
SET(TARGET_SRC  main.cpp )

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR})

SET(TARGET_LIBRARIES_VARS 
	OSG_LIBRARY 
	OSGGA_LIBRARY 
	OSGDB_LIBRARY 
	OSGUTIL_LIBRARY 
	OSGVIEWER_LIBRARY 
	OPENTHREADS_LIBRARY
	)

SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_LIBDIR})
SETUP_APPLICATION(gwNearFarTest)