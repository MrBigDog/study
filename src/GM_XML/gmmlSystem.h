#ifndef __SYSTEM_INFO_H__
#define __SYSTEM_INFO_H__

#include "export.h"

GEOXML_EXPORT void gmmlDisplayErrorandWarningText(const char* textstr);
GEOXML_EXPORT void gmmlStartProgress(int  progressmax);
GEOXML_EXPORT void gmmlShowProgress(const char* textstr);
GEOXML_EXPORT void gmmlProgressing();
GEOXML_EXPORT void gmmlStopProgress();

#endif