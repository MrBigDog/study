#pragma once
#include "vtkUnstructuredGrid.h"
#include<string>
#include<vector>

struct Pillar
{
	Pillar() {};
	Pillar(const Pillar& v)
	{
		HeadPos[0] = v.HeadPos[0];
		HeadPos[1] = v.HeadPos[1];
		HeadPos[2] = v.HeadPos[2];

		TailPos[0] = v.TailPos[0];
		TailPos[1] = v.TailPos[1];
		TailPos[2] = v.TailPos[2];

	}
	Pillar& operator =  (const Pillar& v)//赋值
	{
		HeadPos[0] = v.HeadPos[0];
		HeadPos[1] = v.HeadPos[1];
		HeadPos[2] = v.HeadPos[2];

		TailPos[0] = v.TailPos[0];
		TailPos[1] = v.TailPos[1];
		TailPos[2] = v.TailPos[2];
	}
	double HeadPos[3];
	double TailPos[3];
};

class vtkDataArray;
class VTKEXTENDING_EXPORT GMCornerPointGrid : public vtkUnstructuredGrid
{
public:
	GMCornerPointGrid(void);
	~GMCornerPointGrid(void);
	vtkTypeRevisionMacro(GMCornerPointGrid, vtkUnstructuredGrid);
	static GMCornerPointGrid *New();
	std::string GetID();
	void SetID(std::string);

	//--------
	std::string                  ZValueType; // "elevation"  or "length"
	std::string GetZValueType();
	void SetZValueType(std::string ztype);//// "elevation"  or "length"


	//-------------角点网格的体元集合。设角点网格的维度为 (Ni, Nj, Nk)，则其所包含的体元总数为 (Ni * Nj * Nk)，此即为本子元素所包含元素的数目。
	//	以I方向为“行”、J方向为“列”、K方向为“层”，本元素按行-列-层优先的顺序顺次记录体元的位置。即先记录索引值为(Xi,0,0)（其中Xi为整数，属于区间[0,Ni)）的体元，接着记录索引值为(Xi,1,0)的体元，……，接着记录索引值为(Xi,0,1)的体元，……最后记录索引值为(Xi,Nj - 1, Nk - 1)的体元。

	//角点网格的六面体体元。本元素出现的次数为 (Ni * Nj * Nk)。
	//	角点网格中每个六面体体元都由其顶面4个角点及底面4个角点来确定。而这些角点均位于Pillar线上，因此仅需记录各角点的Z坐标值或其在对应Pillar线上距离Pillar线首顶点的距离即可确定角点的位置。
	//	若在坐标系I-J-K内平移该体元，使其一顶点与原点重合，并使得过该顶点的三条棱分别与对应的坐标轴的正方向重合，然后分别以该三条棱的长度为单位长度，则可以记该体元的8个角点的坐标为(0,0,0)、(1,0,0)、(0,1,0)、(1,1,0)、(0,0,1)、(1,0,1)、(0,1,1)、(1,1,1)。本元素按此顺序记录各角点的海拔高程值（Z坐标）或与Pillar线首顶点的距离。若记体元的索引值为(i,j,k)（其中i,j,k均为从0开始的索引值），则上述8个角点各自所对应的Pillar线的索引值分别为：(i,j)、(i + 1,j)、(i,j + 1)、(i + 1,j + 1)、(i,j)、(i + 1,j)、(i,j + 1)、(i + 1,j + 1)。</documentation>
	void AddCell(double cell[8], char valid);
	void GetCell(int index, double cell[8]);
	void GetCell(int ni, int nj, int nk, double cell[8]);
	int  GetCellCount();
	//-------------------------


	//-------------角点网格的Pillar线组成的数组。Pillar线是沿K轴延展的直线段。
	//	设该角点网格的维度为 (Ni, Nj, Nk)，并以I方向为“行”、以J方向为“列”，则网格的Pillar线可记为 (Nj + 1) 行、(Ni + 1) 列，总条数为 (Nj + 1) * (Ni + 1)。
	//	本元素以“行”优先的方式记录所有的Pillar线，即先记录第0行包含的第0列至第Ni列的共 (Ni + 1) 条Pillar线，然后记录第1行包含的第0列至第Ni列的共 (Ni + 1) 条Pillar线，……，最后记录第Nj行包含的第0列至第Ni列的共 (Ni + 1) 条Pillar线。
	void AddPillar(double HeadPos[3], double TailPos[3]); // 

	void GetPillar(int index, double HeadPos[3], double TailPos[3]);
	void GetPillar(int ni, int nj, double HeadPos[3], double TailPos[3]);
	int  GetPillarCount();


	void AddCellPropertyData(vtkDataArray*);
	vtkDataArray* GetCellPropertyData(int i);
	int GetCountOfCellPropertyData();
	std::vector<char>            ValidCells; // NumberofCells = Cells.size();

	//---------------------------
	void UpdateGrid();
	void GetDimension(int dimension[3]);
	void SetDimension(int dimension[3]);

	std::vector<double>red;
	std::vector<double>gre;
	std::vector<double>blue;

	int rendermode;
	//void ReadFile(char* filename);
protected:
	std::string         _ID;
	int                 Dimension[3];//Ni*Nj*Nk;
	std::vector<Pillar>       Pillars; //Pillars.size() = Ni*Nj;
	std::vector<double>          Cells; // NumberofCells = Cells.size()/8;



	std::vector<vtkDataArray*>   CellPropertyData; // 

};

