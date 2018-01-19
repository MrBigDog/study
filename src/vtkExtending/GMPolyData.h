#pragma once
#include "Export.h"
#include "vtkpolydata.h"
#include<string>
class VTKEXTENDING_EXPORT GMPolyData : public vtkPolyData
{
public:

	GMPolyData(void);
	~GMPolyData(void);
	vtkTypeRevisionMacro(GMPolyData, vtkPolyData);
	static GMPolyData *New();
	void ReadFile(char* filename);
	std::string GetID();
	void SetID(std::string);
	int GetGeometryType();
	void SetGeometryType(int i);
	void SetDBID(int id);
	int GetDBID();
protected:
	std::string         _ID;
	int                 db_id;
	int                 _GeometryType; // 1:Point	3:LineString	5:TIN	10:TEN					
};

