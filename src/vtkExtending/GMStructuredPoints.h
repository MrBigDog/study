#pragma once
#include "vtkStructuredPoints.h"
#include<string>
#include "vtkMatrix4x4.h"
class VTKEXTENDING_EXPORT GMStructuredPoints : public vtkStructuredPoints
{
public:
	GMStructuredPoints(void);
	~GMStructuredPoints(void);

	vtkTypeRevisionMacro(GMStructuredPoints, vtkStructuredPoints);
	static GMStructuredPoints *New();
	std::string GetID();
	void SetID(std::string);
	vtkMatrix4x4*  GetTransformMatrix();
	void CreateTransformMatrix();
	void ReadFile(char*);
protected:
	std::string         _ID;
	vtkMatrix4x4*       _TransformMatrix;

	//std::string         _Labels[3];
	//std::string         _Uom;
};

