/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkImageIterator.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkImageIterator.txx"

#ifndef VTK_NO_EXPLICIT_TEMPLATE_INSTANTIATION

template class VTKEXTENDING_EXPORT vtkImageIterator<signed char>;
template class VTKEXTENDING_EXPORT vtkImageIterator<char>;
template class VTKEXTENDING_EXPORT vtkImageIterator<int>;
template class VTKEXTENDING_EXPORT vtkImageIterator<long>;
template class VTKEXTENDING_EXPORT vtkImageIterator<short>;
template class VTKEXTENDING_EXPORT vtkImageIterator<float>;
template class VTKEXTENDING_EXPORT vtkImageIterator<double>;
template class VTKEXTENDING_EXPORT vtkImageIterator<unsigned long>;
template class VTKEXTENDING_EXPORT vtkImageIterator<unsigned short>;
template class VTKEXTENDING_EXPORT vtkImageIterator<unsigned char>;
template class VTKEXTENDING_EXPORT vtkImageIterator<unsigned int>;
#if defined(VTK_TYPE_USE_LONG_LONG)
template class VTKEXTENDING_EXPORT vtkImageIterator<long long>;
template class VTKEXTENDING_EXPORT vtkImageIterator<unsigned long long>;
#endif
#if defined(VTK_TYPE_USE___INT64)
template class VTKEXTENDING_EXPORT vtkImageIterator<__int64>;
template class VTKEXTENDING_EXPORT vtkImageIterator<unsigned __int64>;
#endif

#endif
