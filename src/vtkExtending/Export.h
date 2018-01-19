#ifndef VTKEXTENDING_EXPORT_H
#define VTKEXTENDING_EXPORT_H

//#ifndef __VTK_SYSTEM_INCLUDES__INSIDE
//Do_not_include_vtkWin32Header_directly__vtkSystemIncludes_includes_it;
//#endif

#include "vtkConfigure.h"

//
// Windows specific stuff------------------------------------------
#if defined(_WIN32) || defined(WIN32) || defined(__CYGWIN__)

// define strict header for windows
#ifndef STRICT
#define STRICT
#endif

#ifdef VTK_USE_ANSI_STDLIB
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#endif

// Never include the windows header here when building VTK itself.
#if defined(VTK_IN_VTK)
# undef VTK_INCLUDE_WINDOWS_H
#endif

#if defined(_WIN32)
// Include the windows header here only if requested by user code.
# if defined(VTK_INCLUDE_WINDOWS_H)
#  include <windows.h>
// Define types from the windows header file.
typedef DWORD vtkWindowsDWORD;
typedef PVOID vtkWindowsPVOID;
typedef LPVOID vtkWindowsLPVOID;
typedef HANDLE vtkWindowsHANDLE;
typedef LPTHREAD_START_ROUTINE vtkWindowsLPTHREAD_START_ROUTINE;
# else
// Define types from the windows header file.
typedef unsigned long vtkWindowsDWORD;
typedef void* vtkWindowsPVOID;
typedef vtkWindowsPVOID vtkWindowsLPVOID;
typedef vtkWindowsPVOID vtkWindowsHANDLE;
typedef vtkWindowsDWORD(__stdcall *vtkWindowsLPTHREAD_START_ROUTINE)(vtkWindowsLPVOID);
# endif
// Enable workaround for windows header name mangling.
// See VTK/Utilities/Upgrading/README.WindowsMangling.txt for details.
# define VTK_WORKAROUND_WINDOWS_MANGLE
#endif

#if defined(_MSC_VER)
// Enable MSVC compiler warning messages that are useful but off by default.
# pragma warning ( default : 4263 ) /* no override, call convention differs */
// Disable MSVC compiler warning messages that often occur in valid code.
# if !defined(VTK_DISPLAY_WIN32_WARNINGS)
#  pragma warning ( disable : 4097 ) /* typedef is synonym for class */
#  pragma warning ( disable : 4127 ) /* conditional expression is constant */
#  pragma warning ( disable : 4244 ) /* possible loss in conversion */
#  pragma warning ( disable : 4251 ) /* missing DLL-interface */
#  pragma warning ( disable : 4305 ) /* truncation from type1 to type2 */
#  pragma warning ( disable : 4309 ) /* truncation of constant value */
#  pragma warning ( disable : 4514 ) /* unreferenced inline function */
#  pragma warning ( disable : 4706 ) /* assignment in conditional expression */
#  pragma warning ( disable : 4710 ) /* function not inlined */
#  pragma warning ( disable : 4786 ) /* identifier truncated in debug info */
# endif
#endif

// MSVC 6.0 in release mode will warn about code it produces with its
// optimizer.  Disable the warnings specifically for this
// configuration.  Real warnings will be revealed by a debug build or
// by other compilers.
#if defined(_MSC_VER) && (_MSC_VER < 1300) && defined(NDEBUG)
# pragma warning ( disable : 4701 ) /* Variable may be used uninitialized.  */
# pragma warning ( disable : 4702 ) /* Unreachable code.  */
#endif


#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)

#	undef VTKEXTENDING_EXPORT

#  if defined(VTKEXTENDING_LIBRARY_STATIC)
#    define VTKEXTENDING_EXPORT
#  elif defined(VTKEXTENDING_LIBRARY)
#    define VTKEXTENDING_EXPORT   __declspec(dllexport)
#  else
#    define VTKEXTENDING_EXPORT  __declspec(dllimport)// __declspec(dllimport)
#  endif
#else
#  define VTKEXTENDING_EXPORT
#endif  

// set up define for whether member templates are supported by VisualStudio compilers.
#ifdef _MSC_VER
# if (_MSC_VER >= 1300)
#  define __STL_MEMBER_TEMPLATES
# endif
#endif

/* Define NULL pointer value */

#ifndef NULL
#ifdef  __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif

#endif