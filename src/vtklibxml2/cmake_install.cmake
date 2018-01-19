# Install script for directory: F:/vtk/vtk5/Utilities/vtklibxml2

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "C:/Program Files/VTK")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vtk-5.4" TYPE STATIC_LIBRARY FILES "F:/vtk/vtk510/bin/Debug/vtklibxml2.lib")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vtk-5.4" TYPE STATIC_LIBRARY FILES "F:/vtk/vtk510/bin/Release/vtklibxml2.lib")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vtk-5.4" TYPE STATIC_LIBRARY FILES "F:/vtk/vtk510/bin/MinSizeRel/vtklibxml2.lib")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vtk-5.4" TYPE STATIC_LIBRARY FILES "F:/vtk/vtk510/bin/RelWithDebInfo/vtklibxml2.lib")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/vtk-5.4/vtklibxml2/libxml" TYPE FILE FILES
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlversion.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/c14n.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/catalog.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/chvalid.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/debugXML.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/dict.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/DOCBparser.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/encoding.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/entities.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/globals.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/hash.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/HTMLparser.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/HTMLtree.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/list.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/nanoftp.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/nanohttp.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/parser.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/parserInternals.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/pattern.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/relaxng.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/SAX.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/SAX2.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/schemasInternals.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/schematron.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/threads.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/tree.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/uri.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/valid.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/vtk_libxml2_mangle.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xinclude.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xlink.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlautomata.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlerror.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlexports.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlIO.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlmemory.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlmodule.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlreader.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlregexp.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlsave.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlschemas.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlschemastypes.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlstring.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlunicode.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xmlwriter.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xpath.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xpathInternals.h"
    "F:/vtk/vtk510/Utilities/vtklibxml2/libxml/xpointer.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Development")

