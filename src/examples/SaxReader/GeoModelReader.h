#ifndef GeoModelReader_h__
#define GeoModelReader_h__

//#include "Export.h"

#include "global.h"
#include "GeologicFeatureClassReader.h"

/*SAXREADER_EXPORT*/ int ReadGeoModel(void* pData, const char* pGeoModelFile, int binary = 0);
/*SAXREADER_EXPORT*/ int ReadMetaData(void* pData, xmlTextReaderPtr reader);
//int ReadFeatureClasses(void* pData,xmlTextReaderPtr reader);
//int ReadFeatureRelationship(void* pData,xmlTextReaderPtr reader);

#endif // GeoModelReader_h__
