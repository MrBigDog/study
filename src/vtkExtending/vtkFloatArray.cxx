/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkFloatArray.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// Instantiate superclass first to give the template a DLL interface.
#include "vtkDataArrayTemplate.txx"
VTK_DATA_ARRAY_TEMPLATE_INSTANTIATE(float);

#define __vtkFloatArray_cxx
#include "vtkFloatArray.h"

#include "vtkObjectFactory.h"

//----------------------------------------------------------------------------
vtkCxxRevisionMacro(vtkFloatArray, "$Revision: 1.58 $");
vtkStandardNewMacro(vtkFloatArray);

//----------------------------------------------------------------------------
vtkFloatArray::vtkFloatArray(vtkIdType numComp): RealSuperclass(numComp)
{
}

//----------------------------------------------------------------------------
vtkFloatArray::~vtkFloatArray()
{
}

//----------------------------------------------------------------------------
void vtkFloatArray::PrintSelf(ostream& os, vtkIndent indent)
{
  this->RealSuperclass::PrintSelf(os,indent);
}