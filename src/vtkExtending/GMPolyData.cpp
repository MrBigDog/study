#include "GMPolyData.h"


#include "vtkCellArray.h"
#include "vtkFieldData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
//#include "vtkPolyDataReader.h"

#include <ctype.h>
#include <sys/stat.h>

vtkCxxRevisionMacro(GMPolyData, "$Revision: 1.8.6.1 $");
GMPolyData::GMPolyData(void)
{
}


GMPolyData::~GMPolyData(void)
{
}

GMPolyData* GMPolyData::New(void)
{
	return new GMPolyData;
}
void GMPolyData::ReadFile(char* filename)
{/*
	vtkPolyDataReader * reader = vtkPolyDataReader::New();

	reader->SetFileName(filename);
	reader->SetOutput(this);
	reader->Update();

	this->SetID("new_Geo_shape_ID_1");
	reader->Delete();
	
	if(this->GetNumberOfCells())
	SetGeometryType(this->GetCellType(0));*/
}

	std::string GMPolyData::GetID()
		{
		
			return _ID;
	}

	void GMPolyData::SetDBID(int id)
	{
		db_id = id;
	}
	int GMPolyData::GetDBID()
	{
		return db_id;
	}
	void GMPolyData::SetID(std::string str)
	{
		_ID = str;
	}

	int GMPolyData::GetGeometryType()
	{
		return _GeometryType;
	}
	void GMPolyData::SetGeometryType(int i)
	{_GeometryType = i;};
