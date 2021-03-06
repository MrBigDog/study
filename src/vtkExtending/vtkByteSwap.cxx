/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkByteSwap.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkByteSwap.h"
#include <memory.h>
#include "vtkObjectFactory.h"

vtkCxxRevisionMacro(vtkByteSwap, "$Revision: 1.50 $");
vtkStandardNewMacro(vtkByteSwap);

//----------------------------------------------------------------------------
vtkByteSwap::vtkByteSwap()
{
}

//----------------------------------------------------------------------------
vtkByteSwap::~vtkByteSwap()
{
}

//----------------------------------------------------------------------------
// Define swap functions for each type size.
template <size_t s> struct vtkByteSwapper;
VTK_TEMPLATE_SPECIALIZE struct vtkByteSwapper<1>
{
  static inline void Swap(void*) {}
};
VTK_TEMPLATE_SPECIALIZE struct vtkByteSwapper<2>
{
  static inline void Swap(void* p)
    {
    char one_byte;
    char* data = static_cast<char*>(p);
    one_byte = data[0]; data[0] = data[1]; data[1] = one_byte;
    }
};
VTK_TEMPLATE_SPECIALIZE struct vtkByteSwapper<4>
{
  static inline void Swap(void* p)
    {
    char one_byte;
    char* data = static_cast<char*>(p);
    one_byte = data[0]; data[0] = data[3]; data[3] = one_byte;
    one_byte = data[1]; data[1] = data[2]; data[2] = one_byte;
    }
};
VTK_TEMPLATE_SPECIALIZE struct vtkByteSwapper<8>
{
  static inline void Swap(void* p)
    {
    char one_byte;
    char* data = static_cast<char*>(p);
    one_byte = data[0]; data[0] = data[7]; data[7] = one_byte;
    one_byte = data[1]; data[1] = data[6]; data[6] = one_byte;
    one_byte = data[2]; data[2] = data[5]; data[5] = one_byte;
    one_byte = data[3]; data[3] = data[4]; data[4] = one_byte;
    }
};

//----------------------------------------------------------------------------
// Define range swap functions.
template <class T> inline void vtkByteSwapRange(T* first, vtkIdType num)
{
  // Swap one value at a time.
  T* last = first + num;
  for(T* p=first; p != last; ++p)
    {
    vtkByteSwapper<sizeof(T)>::Swap(p);
    }
}
inline void vtkByteSwapRangeWrite(const char* first, vtkIdType num,
                                  FILE* f, int)
{
  // No need to swap segments of 1 byte.
  fwrite(first, sizeof(char), num, f);
}
inline void vtkByteSwapRangeWrite(const signed char* first, vtkIdType num,
                                  FILE* f, int)
{
  // No need to swap segments of 1 byte.
  fwrite(first, sizeof(signed char), num, f);
}
inline void vtkByteSwapRangeWrite(const unsigned char* first, vtkIdType num,
                                  FILE* f, int)
{
  // No need to swap segments of 1 byte.
  fwrite(first, sizeof(unsigned char), num, f);
}
template <class T>
inline void vtkByteSwapRangeWrite(const T* first, vtkIdType num, FILE* f, long)
{
  // Swap and write one value at a time.  We do not need to do this in
  // blocks because the file stream is already buffered.
  const T* last = first + num;
  for(const T* p=first; p != last; ++p)
    {
    T temp = *p;
    vtkByteSwapper<sizeof(T)>::Swap(&temp);
    fwrite(&temp, sizeof(temp), 1, f);
    }
}
inline void vtkByteSwapRangeWrite(const char* first, vtkIdType num,
                                  ostream* os, int)
{
  // No need to swap segments of 1 byte.
  os->write((char*)first, num*sizeof(char));
}
inline void vtkByteSwapRangeWrite(const signed char* first, vtkIdType num,
                                  ostream* os, int)
{
  // No need to swap segments of 1 byte.
  os->write((char*)first, num*sizeof(signed char));
}
inline void vtkByteSwapRangeWrite(const unsigned char* first, vtkIdType num,
                                  ostream* os, int)
{
  // No need to swap segments of 1 byte.
  os->write((char*)first, num*sizeof(unsigned char));
}
template <class T>
inline void vtkByteSwapRangeWrite(const T* first, vtkIdType num,
                                  ostream* os, long)
{
  // Swap and write one value at a time.  We do not need to do this in
  // blocks because the file stream is already buffered.
  const T* last = first + num;
  for(const T* p=first; p != last; ++p)
    {
    T temp = *p;
    vtkByteSwapper<sizeof(T)>::Swap(&temp);
    os->write((char*)&temp, sizeof(temp));
    }
}

//----------------------------------------------------------------------------
// Define swap functions for each endian-ness.
#if defined(VTK_WORDS_BIGENDIAN)
template <class T> inline void vtkByteSwapBE(T*) {}
template <class T> inline void vtkByteSwapBERange(T*, vtkIdType) {}
template <class T>
inline void vtkByteSwapBERangeWrite(const T* p, vtkIdType num, FILE* f)
{
  fwrite(p, sizeof(T), num, f);
}
template <class T>
inline void vtkByteSwapBERangeWrite(const T* p, vtkIdType num, ostream* os)
{
  os->write((char*)p, sizeof(T)*num);
}
template <class T> inline void vtkByteSwapLE(T* p)
{
  vtkByteSwapper<sizeof(T)>::Swap(p);
}
template <class T> inline void vtkByteSwapLERange(T* p, vtkIdType num)
{
  vtkByteSwapRange(p, num);
}
template <class T>
inline void vtkByteSwapLERangeWrite(const T* p, vtkIdType num, FILE* f)
{
  vtkByteSwapRangeWrite(p, num, f, 1);
}
template <class T>
inline void vtkByteSwapLERangeWrite(const T* p, vtkIdType num, ostream* os)
{
  vtkByteSwapRangeWrite(p, num, os, 1);
}
#else
template <class T> inline void vtkByteSwapBE(T* p)
{
  vtkByteSwapper<sizeof(T)>::Swap(p);
}
template <class T> inline void vtkByteSwapBERange(T* p, vtkIdType num)
{
  vtkByteSwapRange(p, num);
}
template <class T>
inline void vtkByteSwapBERangeWrite(const T* p, vtkIdType num, FILE* f)
{
  vtkByteSwapRangeWrite(p, num, f, 1);
}
template <class T>
inline void vtkByteSwapBERangeWrite(const T* p, vtkIdType num, ostream* os)
{
  vtkByteSwapRangeWrite(p, num, os, 1);
}
template <class T> inline void vtkByteSwapLE(T*) {}
template <class T> inline void vtkByteSwapLERange(T*, vtkIdType) {}
template <class T>
inline void vtkByteSwapLERangeWrite(const T* p, vtkIdType num, FILE* f)
{
  fwrite(p, sizeof(T), num, f);
}
template <class T>
inline void vtkByteSwapLERangeWrite(const T* p, vtkIdType num, ostream* os)
{
  os->write((char*)p, sizeof(T)*num);
}
#endif

//----------------------------------------------------------------------------
#define VTK_BYTE_SWAP_IMPL(T)                                                   \
  void vtkByteSwap::SwapLE(T* p) { vtkByteSwapLE(p); }                          \
  void vtkByteSwap::SwapBE(T* p) { vtkByteSwapBE(p); }                          \
  void vtkByteSwap::SwapLERange(T* p, vtkIdType num)                            \
    { vtkByteSwapLERange(p, num); }                                             \
  void vtkByteSwap::SwapBERange(T* p, vtkIdType num)                            \
    { vtkByteSwapBERange(p, num); }                                             \
  void vtkByteSwap::SwapLERangeWrite(const T* p, vtkIdType num, FILE* file)     \
    { vtkByteSwapLERangeWrite(p, num, file); }                                  \
  void vtkByteSwap::SwapBERangeWrite(const T* p, vtkIdType num, FILE* file)     \
    { vtkByteSwapBERangeWrite(p, num, file); }                                  \
  void vtkByteSwap::SwapLERangeWrite(const T* p, vtkIdType num, ostream* os)    \
    { vtkByteSwapLERangeWrite(p, num, os); }                                    \
  void vtkByteSwap::SwapBERangeWrite(const T* p, vtkIdType num, ostream* os)    \
    { vtkByteSwapBERangeWrite(p, num, os); }
VTK_BYTE_SWAP_IMPL(float)
VTK_BYTE_SWAP_IMPL(double)
VTK_BYTE_SWAP_IMPL(char)
VTK_BYTE_SWAP_IMPL(short)
VTK_BYTE_SWAP_IMPL(int)
VTK_BYTE_SWAP_IMPL(long)
VTK_BYTE_SWAP_IMPL(signed char)
VTK_BYTE_SWAP_IMPL(unsigned char)
VTK_BYTE_SWAP_IMPL(unsigned short)
VTK_BYTE_SWAP_IMPL(unsigned int)
VTK_BYTE_SWAP_IMPL(unsigned long)
#if defined(VTK_IMPL_USE_LONG_LONG)
VTK_BYTE_SWAP_IMPL(long long)
VTK_BYTE_SWAP_IMPL(unsigned long long)
#endif
#if defined(VTK_IMPL_USE___INT64)
VTK_BYTE_SWAP_IMPL(__int64)
VTK_BYTE_SWAP_IMPL(unsigned __int64)
#endif
#undef VTK_BYTE_SWAP_IMPL

#if VTK_SIZEOF_SHORT == 2
typedef short vtkByteSwapType2;
#else
# error "..."
#endif

#if VTK_SIZEOF_INT == 4
typedef int vtkByteSwapType4;
#else
# error "..."
#endif

#if VTK_SIZEOF_DOUBLE == 8
typedef double vtkByteSwapType8;
#else
# error "..."
#endif

//----------------------------------------------------------------------------
#define VTK_BYTE_SWAP_SIZE(S)                                                   \
  void vtkByteSwap::Swap##S##LE(void* p)                                        \
    { vtkByteSwap::SwapLE(static_cast<vtkByteSwapType##S*>(p)); }               \
  void vtkByteSwap::Swap##S##BE(void* p)                                        \
    { vtkByteSwap::SwapBE(static_cast<vtkByteSwapType##S*>(p)); }               \
  void vtkByteSwap::Swap##S##LERange(void* p, int n)                            \
    { vtkByteSwap::SwapLERange(static_cast<vtkByteSwapType##S*>(p), n); }       \
  void vtkByteSwap::Swap##S##BERange(void* p, int n)                            \
    { vtkByteSwap::SwapBERange(static_cast<vtkByteSwapType##S*>(p), n); }       \
  void vtkByteSwap::SwapWrite##S##LERange(const void* p, int n, FILE* f)        \
    { vtkByteSwap::SwapLERangeWrite(static_cast<const vtkByteSwapType##S*>(p),  \
                                    n, f); }                                    \
  void vtkByteSwap::SwapWrite##S##BERange(const void* p, int n, FILE* f)        \
    { vtkByteSwap::SwapBERangeWrite(static_cast<const vtkByteSwapType##S*>(p),  \
                                    n, f); }                                    \
  void vtkByteSwap::SwapWrite##S##LERange(const void* p, int n, ostream* os)    \
    { vtkByteSwap::SwapLERangeWrite(static_cast<const vtkByteSwapType##S*>(p),  \
                                    n, os); }                                   \
  void vtkByteSwap::SwapWrite##S##BERange(const void* p, int n, ostream* os)    \
    { vtkByteSwap::SwapBERangeWrite(static_cast<const vtkByteSwapType##S*>(p),  \
                                    n, os); }
VTK_BYTE_SWAP_SIZE(2)
VTK_BYTE_SWAP_SIZE(4)
VTK_BYTE_SWAP_SIZE(8)
#undef VTK_BYTE_SWAP_SIZE

//----------------------------------------------------------------------------
// Swaps the bytes of a buffer.  Uses an arbitrary word size, but
// assumes the word size is divisible by two.
void vtkByteSwap::SwapVoidRange(void *buffer, int numWords, int wordSize)
{
  unsigned char temp, *out, *buf;
  int idx1, idx2, inc, half;
  
  half = wordSize / 2;
  inc = wordSize - 1;
  buf = (unsigned char *)(buffer);
  
  for (idx1 = 0; idx1 < numWords; ++idx1)
    {
      out = buf + inc;
      for (idx2 = 0; idx2 < half; ++idx2)
        {
          temp = *out;
          *out = *buf;
          *buf = temp;
          ++buf;
          --out;
        }
      buf += half;
    }
}
