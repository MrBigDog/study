SET(TARGET_H
	#Export.h
	FeatureRelationReader.h
	FieldReader.h
	Geo3DLayerReader.h
	Geo3DMapReader.h
	Geo3DProjectReader.h
	Geo3DStyleReader.h
	GeoFeatureReader.h
	GeologicFeatureClassReader.h
	GeoModelReader.h
	global.h
	gmmlDisplayErrorandWarning.h
	gmmlSystem.h
	gmmlSystemInfo.h
	MyTimer.h
	ShapePropertyReader.h
	ShapeReader.h
	#stdafx.h
	targetver.h
	XMLBase64.h
)

SET(TARGET_SRC
	FeatureRelationReader.cpp
	FieldReader.cpp
	Geo3DLayerReader.cpp
	Geo3DMapReader.cpp
	Geo3DProjectReader.cpp
	Geo3DStyleReader.cpp
	GeoFeatureReader.cpp
	GeologicFeatureClassReader.cpp
	GeoModelReader.cpp
	global.cpp
	gmmlDisplayErrorandWarning.cpp
	gmmlSystemInfo.cpp
	SaxReader.cpp
	ShapePropertyReader.cpp
	ShapeReader.cpp
	#stdafx.cpp
	xmlBase64.cpp
)
	
INCLUDE_DIRECTORIES(
	${GWEARTH_SOURCE_DIR}/Src 
	${GWEARTH_SOURCE_DIR}/Src/SaxReader
	${GWEARTH_SOURCE_DIR}/Src/vtklibxml2
	${GWEARTH_SOURCE_DIR}/Src/GM_XML
	${GWEARTH_SOURCE_DIR}/Src/vtkExtending
	${ICONV_INCLUDE_DIR}
	)

LINK_WITH_VARIABLES(${LIB_NAME} ICONV_LIBRARIES )


SET(TARGET_COMMON_LIBRARIES  GMXMLFileIO GMLFeature GM_XML vtklibxml2 vtkExtending msvcprtd)
 
#LINK_INTERNAL(${LIB_NAME} ICONV_LIBRARIES msvcprtd.lib)


SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_LIBDIR})
SETUP_APPLICATION(saxReader)