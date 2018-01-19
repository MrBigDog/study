#pragma once
#include "global.h"

class vtkObject;

/*SAXREADER_EXPORT*/ vtkObject* ReadShape(void* pData, xmlTextReaderPtr reader, int binary = 0);
/*SAXREADER_EXPORT*/ vtkObject* ReadGeoTin(void* pData, xmlTextReaderPtr reader, int binary = 0);

/*SAXREADER_EXPORT*/ int ReadVertices(void* pData, xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadTriangles(void* pData, xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ vtkObject* ReadPoint(xmlTextReaderPtr reader, int binary = 0);
/*SAXREADER_EXPORT*/ vtkObject* ReadMultiPoint(xmlTextReaderPtr reader, int binary = 0);
/*SAXREADER_EXPORT*/ vtkObject* ReadLineString(xmlTextReaderPtr reader, int binary = 0);
/*SAXREADER_EXPORT*/ vtkObject* ReadMultiLineString(xmlTextReaderPtr reader, int binary = 0);
/*SAXREADER_EXPORT*/ vtkObject* ReadGeoVolume(xmlTextReaderPtr reader, int binary = 0);
/*SAXREADER_EXPORT*/ vtkObject* ReadGeoGrid(xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ vtkObject* ReadGeoCornerPointGrid(xmlTextReaderPtr reader, int binary);

/*SAXREADER_EXPORT*/ int ReadRectifiedGrid(void* pData, xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadPillars(void* pData, xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadCCells(void* pData, xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadGridEnvelope(void* pData, xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadPos(void* pData, xmlTextReaderPtr reader);

