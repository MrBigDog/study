#pragma once

#include "global.h"

/*SAXREADER_EXPORT*/ int ReadGeo3DStyle(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadFilter(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadCovergeFilter(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadSymbolizer(void* pData,xmlTextReaderPtr reader,const char* pSymbolizer=NULL);
/*SAXREADER_EXPORT*/ int ReadCovergeSymbolizer(void* pData,xmlTextReaderPtr reader,const char* pSymbolizer=NULL);
/*SAXREADER_EXPORT*/ int ReadGeofaceSymbolizer(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadGeoLineSymbolizer(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadPointSymbolizer(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadMaterial(void* pData,xmlTextReaderPtr reader);

/*SAXREADER_EXPORT*/ int ReadGeoDiscreteCoverageSymbolizer(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadRasterSymbolizer(void* pData,xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ int ReadGeoReferencedTexture(void* pData,xmlTextReaderPtr reader);