IF   (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DVTKLIBXML2)
ELSE (DYNAMIC_GWEARTH)
    ADD_DEFINITIONS(-DVTKLIBXML2_STATIC)
ENDIF(DYNAMIC_GWEARTH)

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
ENDIF(WIN32)

SET(LIB_NAME "vtklibxml2")

SET(HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME})
#INCLUDE_DIRECTORIES(AFTER ${OSG_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(AFTER ${OSGEARTH_INCLUDE_DIR})

SET(LIB_PUBLIC_HEADERS
libxml.h
config.h
elfgcchack.h
trio.h
triodef.h
trionan.h
triop.h
triostr.h
libxml/c14n.h
libxml/catalog.h
libxml/chvalid.h
libxml/debugXML.h
libxml/dict.h
libxml/DOCBparser.h
libxml/encoding.h
libxml/entities.h
libxml/globals.h
libxml/hash.h
libxml/HTMLparser.h
libxml/HTMLtree.h
libxml/list.h
libxml/nanoftp.h
libxml/nanohttp.h
libxml/parser.h
libxml/parserInternals.h
libxml/pattern.h
libxml/relaxng.h
libxml/SAX.h
libxml/SAX2.h
libxml/schemasInternals.h
libxml/schematron.h
libxml/threads.h
libxml/tree.h
libxml/uri.h
libxml/valid.h
libxml/vssver2.scc
libxml/vtk_libxml2_mangle.h
libxml/xinclude.h
libxml/xlink.h
libxml/xmlautomata.h
libxml/xmlerror.h
libxml/xmlexports.h
libxml/xmlIO.h
libxml/xmlmemory.h
libxml/xmlmodule.h
libxml/xmlreader.h
libxml/xmlregexp.h
libxml/xmlsave.h
libxml/xmlschemas.h
libxml/xmlschemastypes.h
libxml/xmlstring.h
libxml/xmlunicode.h
libxml/xmlversion.h
libxml/xmlversion.h.in
libxml/xmlwin32version.h.in
libxml/xmlwriter.h
libxml/xpath.h
libxml/xpathInternals.h
libxml/xpointer.h
)

SET(TARGET_SRC
	DOCBparser.c
	HTMLparser.c
	HTMLtree.c
	SAX.c
	SAX2.c
	c14n.c
	catalog.c
	chvalid.c
	debugXML.c
	dict.c
	encoding.c
	entities.c
	error.c
	globals.c
	hash.c
	legacy.c
	list.c
	nanoftp.c
	nanohttp.c
	parser.c
	parserInternals.c
	pattern.c
	relaxng.c
	schematron.c
	threads.c
	tree.c
	uri.c
	valid.c
	xinclude.c
	xlink.c
	xmlIO.c
	xmlmemory.c
	xmlmodule.c
	xmlreader.c
	xmlregexp.c
	xmlsave.c
	xmlschemas.c
	xmlschemastypes.c
	xmlstring.c
	xmlunicode.c
	xmlwriter.c
	xpath.c
	xpointer.c
	platformTestsC.c
	rngparser.c
	#runsuite.c
	#runtest.c
	trio.c
	trionan.c
	triostr.c
	vtk_libxml2_zlib.h
	xmlcatalog.c
)

ADD_LIBRARY(
    ${LIB_NAME} 
	${GWEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TARGET_SRC} )
	
INCLUDE_DIRECTORIES(${GWEARTH_SOURCE_DIR}/Src ${GWEARTH_SOURCE_DIR}/Src/vtklibxml2 ${ICONV_INCLUDE_DIR})

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

LINK_WITH_VARIABLES(${LIB_NAME} ICONV_LIBRARIES	)
 
#LINK_INTERNAL(${LIB_NAME} shlwapi)
#LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)