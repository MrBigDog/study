#ifndef GWGEOLOGICALUTIL_EXPORT_H
#define GWGEOLOGICALUTIL_EXPORT_H 1

#if defined(_MSC_VER)
#pragma warning( disable : 4244 )
#pragma warning( disable : 4251 )
#pragma warning( disable : 4267 )
#pragma warning( disable : 4275 )
#pragma warning( disable : 4290 )
#pragma warning( disable : 4786 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4996 )
#endif

#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
#  if defined( GWGEOLOGICALUTIL_LIBRARY_STATIC )
#    define GWGEOLOGICALUTIL_EXPORT
#  elif defined( GWGEOLOGICALUTIL_LIBRARY )
#    define GWGEOLOGICALUTIL_EXPORT   __declspec(dllexport)
#  else
#    define GWGEOLOGICALUTIL_EXPORT   __declspec(dllimport)
#  endif
#else
#  define GWGEOLOGICALUTIL_EXPORT
#endif  



#endif