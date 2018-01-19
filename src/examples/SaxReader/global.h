#ifndef GLOBAL_H
#define GLOBAL_H

//#include "Export.h"
#include <libxml/xmlreader.h>

/*SAXREADER_EXPORT*/ char* ConvertEnc(char* encFrom, char* encTo, char* in);
/*SAXREADER_EXPORT*/ char* ReadValueAndConvert(xmlTextReaderPtr reader);
/*SAXREADER_EXPORT*/ char* ReadAttrAndConvert(xmlTextReaderPtr reader,const char* attribute_name);

/*SAXREADER_EXPORT*/ int TrimLeft(char* DocName);
/*SAXREADER_EXPORT*/ int TrimRight(char* DocName);

/*SAXREADER_EXPORT*/ int InitCharSet(const char* pFileName);

 extern /*SAXREADER_EXPORT*/ char gCharSet[256];

#endif