#pragma once

#define EXPORT_LIB // Ç¿ÖÆÎª¾²Ì¬¿â



#ifdef EXPORT_LIB
#define DLL_OPTION

#else
#ifdef EXPORT_DLL

#include <string>
#include <vector>

using namespace std;

#define DLL_OPTION __declspec(dllexport)  
#else  
#define DLL_OPTION __declspec(dllimport)  
#endif

#endif
