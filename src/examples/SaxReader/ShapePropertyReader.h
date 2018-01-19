#pragma once

#include "global.h"
#include <vector>

class vtkDataArray;

/*SAXREADER_EXPORT*/ int ReadShapeProperty(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadDiscreteCoverage(xmlTextReaderPtr reader, std::vector<vtkDataArray*> &dataarrays,std::vector<int> &dataarraytypes, std::vector<std::string>& dataarraysname);
/*SAXREADER_EXPORT*/ int ReadDiscreteCoverageGMML(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ vtkDataArray* ReadDataArray(xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ vtkDataArray* ReadValueArray(xmlTextReaderPtr reader);
