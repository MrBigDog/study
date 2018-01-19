#include "GMUnstructuredGrid.h"

#include "vtkCellArray.h"
#include "vtkFieldData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
#include "vtkCharArray.h"
//#include "vtkUnstructuredGridreader.h"

#include <ctype.h>
#include <sys/stat.h>

vtkCxxRevisionMacro(GMUnstructuredGrid, "$Revision: 1.8.6.1 $");
GMUnstructuredGrid::GMUnstructuredGrid(void)
{
}


GMUnstructuredGrid::~GMUnstructuredGrid(void)
{
}

GMUnstructuredGrid* GMUnstructuredGrid::New(void)
{
	return new GMUnstructuredGrid;
}
std::string GMUnstructuredGrid::GetID()
{
	return _ID;
}
void GMUnstructuredGrid::SetID(std::string str)
{
	_ID = str;
}

int GMUnstructuredGrid::GetGeometryType()
{
	return _GeometryType;
}
void GMUnstructuredGrid::SetGeometryType(int i)
{
	_GeometryType = i;
};

void GMUnstructuredGrid::ReadFile(char* filename)
{/*
	vtkUnstructuredGridReader * reader = vtkUnstructuredGridReader::New();

	reader->SetFileName(filename);
	reader->SetOutput(this);
	reader->Update();

	this->SetID("new_Geo_Volume_ID_1");
	reader->Delete();

	if(this->GetNumberOfCells())
SetGeometryType(this->GetCellType(0));*/
}
