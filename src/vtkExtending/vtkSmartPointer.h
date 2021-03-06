/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkSmartPointer.h,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkSmartPointer - Hold a reference to a vtkObjectBase instance.
// .SECTION Description
// vtkSmartPointer is a class template that provides automatic casting
// for objects held by the vtkSmartPointerBase superclass.

#ifndef __vtkSmartPointer_h
#define __vtkSmartPointer_h

#include "vtkSmartPointerBase.h"

template <class T>
class vtkSmartPointer: public vtkSmartPointerBase
{
public:
  // Description:
  // Initialize smart pointer to NULL.
  vtkSmartPointer() {}

  // Description:
  // Initialize smart pointer to given object.
  vtkSmartPointer(T* r): vtkSmartPointerBase(r) {}

  // Description:
  // Initialize smart pointer with a new reference to the same object
  // referenced by given smart pointer.
  vtkSmartPointer(const vtkSmartPointerBase& r): vtkSmartPointerBase(r) {}

  // Description:
  // Assign object to reference.  This removes any reference to an old
  // object.
  vtkSmartPointer& operator=(T* r)
    {
    this->vtkSmartPointerBase::operator=(r);
    return *this;
    }

  // Description:
  // Assign object to reference.  This removes any reference to an old
  // object.
  vtkSmartPointer& operator=(const vtkSmartPointerBase& r)
    {
    this->vtkSmartPointerBase::operator=(r);
    return *this;
    }

  // Description:
  // Get the contained pointer.
  T* GetPointer() const
    {
    return static_cast<T*>(this->Object);
    }

  // Description:
  // Get the contained pointer.
  operator T* () const
    {
    return static_cast<T*>(this->Object);
    }

  // Description:
  // Dereference the pointer and return a reference to the contained
  // object.
  T& operator*() const
    {
    return *static_cast<T*>(this->Object);
    }

  // Description:
  // Provides normal pointer target member access using operator ->.
  T* operator->() const
    {
    return static_cast<T*>(this->Object);
    }

  // Description:
  // Transfer ownership of one reference to the given VTK object to
  // this smart pointer.  This does not increment the reference count
  // of the object, but will decrement it later.  The caller is
  // effectively passing ownership of one reference to the smart
  // pointer.  This is useful for code like:
  //
  //   vtkSmartPointer<vtkFoo> foo;
  //   foo.TakeReference(bar->NewFoo());
  //
  // The input argument may not be another smart pointer.
  void TakeReference(T* t)
    {
    *this = vtkSmartPointer<T>(t, NoReference());
    }

  // Description:
  // Create an instance of a VTK object.
  static vtkSmartPointer<T> New()
    {
    return vtkSmartPointer<T>(T::New(), NoReference());
    }

  // Description:
  // Create a new instance of the given VTK object.
  static vtkSmartPointer<T> NewInstance(T* t)
    {
    return vtkSmartPointer<T>(t->NewInstance(), NoReference());
    }

  // Description:
  // Transfer ownership of one reference to the given VTK object to a
  // new smart pointer.  The returned smart pointer does not increment
  // the reference count of the object on construction but will
  // decrement it on destruction.  The caller is effectively passing
  // ownership of one reference to the smart pointer.  This is useful
  // for code like:
  //
  //   vtkSmartPointer<vtkFoo> foo =
  //     vtkSmartPointer<vtkFoo>::Take(bar->NewFoo());
  //
  // The input argument may not be another smart pointer.
  static vtkSmartPointer<T> Take(T* t)
    {
    return vtkSmartPointer<T>(t, NoReference());
    }

  // Work-around for HP overload resolution bug.  Since
  // NullPointerOnly is a private type the only pointer value that can
  // be passed by user code is a null pointer.  This operator will be
  // chosen by the compiler when comparing against null explicitly and
  // avoid the bogus ambiguous overload error.
#if defined(__HP_aCC)
# define VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND(op) \
  vtkstd_bool operator op (NullPointerOnly*) const        \
    {                                                     \
    return ::operator op (*this, 0);                      \
    }
private:
  class NullPointerOnly {};
public:
  VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND(==)
  VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND(!=)
  VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND(<)
  VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND(<=)
  VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND(>)
  VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND(>=)
# undef VTK_SMART_POINTER_DEFINE_OPERATOR_WORKAROUND
#endif
protected:
  vtkSmartPointer(T* r, const NoReference& n): vtkSmartPointerBase(r, n) {}
private:
  // These are purposely not implemented to prevent callers from
  // trying to take references from other smart pointers.
  void TakeReference(const vtkSmartPointerBase&);  // Not implemented.
  static void Take(const vtkSmartPointerBase&);  // Not implemented.
};

#define VTK_SMART_POINTER_DEFINE_OPERATOR(op) \
  template <class T> \
  inline vtkstd_bool \
  operator op (const vtkSmartPointer<T>& l, const vtkSmartPointer<T>& r) \
    { \
    return (l.GetPointer() op r.GetPointer()); \
    } \
  template <class T> \
  inline vtkstd_bool operator op (T* l, const vtkSmartPointer<T>& r) \
    { \
    return (l op r.GetPointer()); \
    } \
  template <class T> \
  inline vtkstd_bool operator op (const vtkSmartPointer<T>& l, T* r) \
    { \
    return (l.GetPointer() op r); \
    }
// Description:
// Compare smart pointer values.
VTK_SMART_POINTER_DEFINE_OPERATOR(==)
VTK_SMART_POINTER_DEFINE_OPERATOR(!=)
VTK_SMART_POINTER_DEFINE_OPERATOR(<)
VTK_SMART_POINTER_DEFINE_OPERATOR(<=)
VTK_SMART_POINTER_DEFINE_OPERATOR(>)
VTK_SMART_POINTER_DEFINE_OPERATOR(>=)

#undef VTK_SMART_POINTER_DEFINE_OPERATOR

// Description:
// Streaming operator to print smart pointer like regular pointers.
template <class T>
inline ostream& operator << (ostream& os, const vtkSmartPointer<T>& p)
{
  return os << static_cast<const vtkSmartPointerBase&>(p);
}

#endif
