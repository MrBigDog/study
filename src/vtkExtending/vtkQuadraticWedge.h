/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkQuadraticWedge.h,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkQuadraticWedge - cell represents a parabolic, 15-node isoparametric wedge
// .SECTION Description
// vtkQuadraticWedge is a concrete implementation of vtkNonLinearCell to
// represent a three-dimensional, 15-node isoparametric parabolic
// wedge. The interpolation is the standard finite element, quadratic
// isoparametric shape function. The cell includes a mid-edge node. The
// ordering of the fifteen points defining the cell is point ids (0-5,6-15)
// where point ids 0-5 are the six corner vertices of the wedge; followed by
// nine midedge nodes (6-15). Note that these midedge nodes correspond lie
// on the edges defined by (0,1), (1,2), (2,0), (3,4), (4,5), (5,3), (0,3),
// (1,4), (2,5).

// .SECTION See Also
// vtkQuadraticEdge vtkQuadraticTriangle vtkQuadraticTetra
// vtkQuadraticHexahedron vtkQuadraticQuad vtkQuadraticPyramid

#ifndef __vtkQuadraticWedge_h
#define __vtkQuadraticWedge_h

#include "vtkNonLinearCell.h"

class vtkQuadraticEdge;
class vtkQuadraticQuad;
class vtkQuadraticTriangle;
class vtkWedge;
class vtkDoubleArray;

class VTKEXTENDING_EXPORT vtkQuadraticWedge : public vtkNonLinearCell
{
public:
  static vtkQuadraticWedge *New();
  vtkTypeRevisionMacro(vtkQuadraticWedge,vtkNonLinearCell);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Implement the vtkCell API. See the vtkCell API for descriptions 
  // of these methods.
  int GetCellType() {return VTK_QUADRATIC_WEDGE;}
  int GetCellDimension() {return 3;}
  int GetNumberOfEdges() {return 9;}
  int GetNumberOfFaces() {return 5;}
  vtkCell *GetEdge(int edgeId);
  vtkCell *GetFace(int faceId);

  int CellBoundary(int subId, double pcoords[3], vtkIdList *pts);
  void Contour(double value, vtkDataArray *cellScalars, 
               vtkPointLocator *locator, vtkCellArray *verts, 
               vtkCellArray *lines, vtkCellArray *polys, 
               vtkPointData *inPd, vtkPointData *outPd,
               vtkCellData *inCd, vtkIdType cellId, vtkCellData *outCd);
  int EvaluatePosition(double x[3], double* closestPoint,
                       int& subId, double pcoords[3], 
                       double& dist2, double *weights);
  void EvaluateLocation(int& subId, double pcoords[3], double x[3],
                        double *weights);
  int Triangulate(int index, vtkIdList *ptIds, vtkPoints *pts);
  void Derivatives(int subId, double pcoords[3], double *values, 
                   int dim, double *derivs);
  virtual double *GetParametricCoords();

  // Description:
  // Clip this quadratic hexahedron using scalar value provided. Like 
  // contouring, except that it cuts the hex to produce linear 
  // tetrahedron.
  void Clip(double value, vtkDataArray *cellScalars, 
            vtkPointLocator *locator, vtkCellArray *tetras,
            vtkPointData *inPd, vtkPointData *outPd,
            vtkCellData *inCd, vtkIdType cellId, vtkCellData *outCd,
            int insideOut);

  // Description:
  // Line-edge intersection. Intersection has to occur within [0,1] parametric
  // coordinates and with specified tolerance.
  int IntersectWithLine(double p1[3], double p2[3], double tol, double& t,
                        double x[3], double pcoords[3], int& subId);

  
  // Description:
  // Return the center of the quadratic wedge in parametric coordinates.
  int GetParametricCenter(double pcoords[3]);

  // Description:
  // Quadratic hexahedron specific methods. 
  static void InterpolationFunctions(double pcoords[3], double weights[15]);
  static void InterpolationDerivs(double pcoords[3], double derivs[45]);

  // Description:
  // Given parametric coordinates compute inverse Jacobian transformation
  // matrix. Returns 9 elements of 3x3 inverse Jacobian plus interpolation
  // function derivatives.
  void JacobianInverse(double pcoords[3], double **inverse, double derivs[45]);

protected:
  vtkQuadraticWedge();
  ~vtkQuadraticWedge();

  vtkQuadraticEdge *Edge;
  vtkQuadraticTriangle *TriangleFace;
  vtkQuadraticQuad *Face;
  vtkWedge         *Wedge;
  vtkPointData     *PointData;
  vtkCellData      *CellData;
  vtkDoubleArray   *CellScalars;
  vtkDoubleArray   *Scalars; //used to avoid New/Delete in contouring/clipping
  
  void Subdivide(vtkPointData *inPd, vtkCellData *inCd, vtkIdType cellId, vtkDataArray *cellScalars);

private:
  vtkQuadraticWedge(const vtkQuadraticWedge&);  // Not implemented.
  void operator=(const vtkQuadraticWedge&);  // Not implemented.
};
//----------------------------------------------------------------------------
// Return the center of the quadratic wedge in parametric coordinates.
inline int vtkQuadraticWedge::GetParametricCenter(double pcoords[3])
{
  pcoords[0] = pcoords[1] = 1./3;
  pcoords[2] = 0.5;
  return 0;
}


#endif


