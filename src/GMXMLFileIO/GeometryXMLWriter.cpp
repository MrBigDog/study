#include "GeometryXMLWriter.h"
#include "../vtkExtending/GMPolyData.h"
#include "../vtkExtending/GMUnstructuredGrid.h"
#include "../vtkExtending/GMStructuredPoints.h"
#include "../vtkExtending/GMCornerPointGrid.h"

#include "vtkCellArray.h"
#include "vtkMath.h"
#include "vtkPoints.h"
#include "vtkPointData.h"
#include "vtkCellData.h"
//#include "vtkPolyDataNormals.h"

#include "XMLBase64.h"

int GeometryXMLWriter::BinaryExport;

GeometryXMLWriter::GeometryXMLWriter(void)
{
}


GeometryXMLWriter::~GeometryXMLWriter(void)
{
}

void GeometryXMLWriter::WriteGeometry(vtkObject* shape, std::ostream& xml_stream)
{
	std::string xml_node_name;
	std::string strShapeName = shape->GetClassName();

	if(strShapeName == "GMPolyData")
	{
		GMPolyData* poly_data = (GMPolyData*)shape;

		if(poly_data && poly_data->GetGeometryType() == 5 )
			xml_node_name = "GeoTin";

		if(poly_data && poly_data->GetGeometryType() == 4 )
			xml_node_name = "gml:MultiCurve";

		if(poly_data && poly_data->GetGeometryType() == 3 )
			xml_node_name = "gml:LineString";

		if(poly_data && poly_data->GetGeometryType() == 1 )
			xml_node_name = "gml:Point";

		//多点数据
		if(poly_data && poly_data->GetGeometryType() == 2 )
			xml_node_name = "gml:MultiPoint";

		if(!BinaryExport)
			xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID() << "\">" << std::endl;
		else
		{
			if(poly_data->GetGeometryType() == 4||poly_data->GetGeometryType() == 2)
			{
				xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID() << "\">" << std::endl;
			}
			else
			{
				xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID() << "\"" << " dt:dt=\"binary.base64\""<< ">" << std::endl;
			}
			
		}

		if(poly_data && poly_data->GetGeometryType() == 5 )
		{
			if(BinaryExport)
				WriteTINBinary(poly_data, xml_stream);
			else
				WriteTIN(poly_data, xml_stream);
		}

		

		if(poly_data && poly_data->GetGeometryType() == 3)
		{
			if(BinaryExport)
				WriteLinestringBinary(poly_data, xml_stream);

			else
				WriteLinestring(  poly_data, xml_stream);
		}



		if(poly_data && poly_data->GetGeometryType() == 4)
		{
			if(BinaryExport)
				WriteMultiLinestring(poly_data, xml_stream);

			else
				WriteMultiLinestring(  poly_data, xml_stream);
		}

		if(poly_data && poly_data->GetGeometryType() == 2)
		{
			if(BinaryExport)
				WriteMultiPoint(poly_data, xml_stream);

			else
				WriteMultiPoint(poly_data, xml_stream);
		}

		if(poly_data && poly_data->GetGeometryType() == 1)
		{
			if(BinaryExport)
				WritePointBinary(poly_data, xml_stream);

			else
				WritePoint(poly_data, xml_stream);
		}
	}
	else
	{
		if(strShapeName == "GMUnstructuredGrid")
		{
			GMUnstructuredGrid *poly_data = (GMUnstructuredGrid *)shape;
			xml_node_name = "GeoVolume";

			if(BinaryExport)
			{
				xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID() << "\"" <<" dt:dt=\"binary.base64\""<< ">" << std::endl;
				WriteGeoVolumeBinary(poly_data,xml_stream);
			}
			else
			{
				xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID()<< "\">" << std::endl;
				WriteGeoVolume(poly_data, xml_stream);
			}
			
		}
		else if(strShapeName == "GMStructuredPoints")
		{
			GMStructuredPoints *poly_data = (GMStructuredPoints *)shape;
			xml_node_name="GeoGrid";
			xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID() << "\">" << std::endl;

			WriteStructedPoints(  poly_data, xml_stream);
		}

		else if(strShapeName == "GMCornerPointGrid")
		{
			GMCornerPointGrid *poly_data = (GMCornerPointGrid *)shape;
			xml_node_name="GeoCornerPointGrid";
			

			if(BinaryExport)
			{
               xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID() << "\"" << " dt:dt=\"binary.base64\""<< ">" << std::endl;
				WriteGeoCornerPointGridBinary(  poly_data, xml_stream);
			}
			else
			{
				xml_stream << "<" << xml_node_name << " gml:id=\"" << poly_data->GetID() << "\">" << std::endl;
				WriteGeoCornerPointGrid(  poly_data, xml_stream);
			}
			
		}

	}

	xml_stream << "</" << xml_node_name << ">" << std::endl;
}

void GeometryXMLWriter::WritePoint(GMPolyData* poly_data,std::ostream& xml_stream)
{
	double xx[3];
	vtkPoints* points  = poly_data->GetPoints();
	long i = 0, vCount = points->GetNumberOfPoints();
	if(vCount) 
		points->GetPoint(0,xx);
	char buf[256] = {'\0'};
	sprintf_s(buf,"%lf   %lf  %lf",xx[0] ,xx[1] ,xx[2] );

	xml_stream << "<gml:pos srsDimension=\"3\">" << buf << "</gml:pos>" << std::endl;
}


void GeometryXMLWriter::WriteMultiPoint(GMPolyData* poly_data,std::ostream& xml_stream)
{
	double xx[3];
	vtkPoints* points  = poly_data->GetPoints();
	long vCount = points->GetNumberOfPoints();
	if(vCount>0)
	{
		for(int i=0;i<vCount;i++)
		{
          xml_stream << "<gml:pointMember>"<< std::endl;
		  xml_stream << "<gml:Point>"<< std::endl;
		  points->GetPoint(i,xx);
		  char buf[256] = {'\0'};
		  sprintf_s(buf,"%lf   %lf  %lf",xx[0] ,xx[1] ,xx[2] );
		  xml_stream << "<gml:pos srsDimension=\"3\">" << buf << "</gml:pos>" << std::endl;
		 
		   xml_stream << "</gml:Point>"<< std::endl;
		   xml_stream << "</gml:pointMember>"<< std::endl;
		}
	}
		
}

void GeometryXMLWriter::WritePointBinary(GMPolyData* poly_data,std::ostream& xml_stream)
{
	vtkPoints* points  = poly_data->GetPoints();
	if(points)
	{
		int vCount = points->GetNumberOfPoints();

		char byteOrder =0;
		int wkbType = 1001;
		int numPoints = vCount;
		char byteOrdercode[sizeof(char)*16];
		int len = Base64Encode(byteOrdercode, (char*)&byteOrder, sizeof(char));		
		xml_stream << byteOrdercode;

		char wkbTypecode[sizeof(int)*4];
		len = Base64Encode(wkbTypecode, (char*)&wkbType, sizeof(int));		
		xml_stream << wkbTypecode;

		char numPointscode[sizeof(int)*4];
		len = Base64Encode(numPointscode, (char*)&numPoints, sizeof(int));		
		xml_stream << numPointscode;

		for (long i = 0; i < 1 && i< numPoints; i++)
		{
			double xx[3];
			points->GetPoint(i,xx);
			char basecode[sizeof(double)*3*4];

			int len1 = Base64Encode(basecode,(char*)xx, sizeof(double)*3);
			xml_stream << basecode;
		}
		xml_stream << std::endl;
	}
}


void GeometryXMLWriter::WriteMultiLinestring(GMPolyData* poly_data,std::ostream& xml_stream)
{
	double xx[3];
	   
	long vCount =  poly_data->GetNumberOfCells();
	if(vCount>0)
	{
		for(int i=0;i<vCount;i++)
		{
			xml_stream << "<gml:curveMember>"<< std::endl;
			xml_stream << "<gml:LineString>"<< std::endl;
			int numpoint=poly_data->GetCell(i)->GetNumberOfPoints();
			if(numpoint>1)
			{
				int Count = numpoint;
				xml_stream  << "<gml:posList srsDimension=\"3\" count=\"" << Count << "\">" << std::endl;
				for (int m = 0; m < Count; m++)
				{
					double xx[3];
				poly_data->GetPoint(poly_data->GetCell(i)->GetPointId(m),xx);
					char buf[256] = {'\0'};
					sprintf_s(buf,"%lf   %lf  %lf",xx[0] ,xx[1] ,xx[2] );
					xml_stream << buf << std::endl;
				}
				xml_stream << "</gml:posList>" << std::endl;
			}
			
			xml_stream << "</gml:LineString>"<< std::endl;
			xml_stream << "</gml:curveMember>"<< std::endl;
		}
	}

}


void GeometryXMLWriter::WriteLinestring(GMPolyData* poly_data,std::ostream& xml_stream)
{
	vtkPoints* points = poly_data->GetPoints();
	if(points)
	{
		int vCount = points->GetNumberOfPoints();;
		xml_stream  << "<gml:posList srsDimension=\"3\" count=\"" << vCount << "\">" << std::endl;
		for (int i = 0; i < vCount; i++)
		{
			double xx[3];
			points->GetPoint(i,xx);
			char buf[256] = {'\0'};
			sprintf_s(buf,"%lf   %lf  %lf",xx[0] ,xx[1] ,xx[2] );
			xml_stream << buf << std::endl;
		}
	}
	xml_stream << "</gml:posList>" << std::endl;
}


void GeometryXMLWriter::WriteLinestringBinary(GMPolyData* poly_data,std::ostream& xml_stream)
{	
	vtkPoints* points  = poly_data->GetPoints();
	if(points){
		int vCount = points->GetNumberOfPoints();;

		char byteOrder = 0;
		int wkbType = 1002;
		int numPoints = vCount;
		char byteOrdercode[sizeof(char)*16];
		int len = Base64Encode(byteOrdercode, (char*)&byteOrder, sizeof(char));		
		xml_stream << byteOrdercode;

		char wkbTypecode[sizeof(int)*4];
		len = Base64Encode(wkbTypecode, (char*)&wkbType, sizeof(int));		
		xml_stream << wkbTypecode;

		char numPointscode[sizeof(int)*4];
		len = Base64Encode(numPointscode, (char*)&numPoints, sizeof(int));		
		xml_stream << numPointscode;

		for (long i = 0; i < vCount; i++)
		{
			double xx[3];
			points->GetPoint(i,xx);
			char basecode[sizeof(double)*3*4];
			int len1 = Base64Encode(basecode, (char*)xx, sizeof(double)*3);
			xml_stream << basecode;
		}
		xml_stream << std::endl;
	}

}

void GeometryXMLWriter::WriteTIN(GMPolyData* poly_data,std::ostream& xml_stream)
{
	WriteVertex(poly_data,xml_stream);
	xml_stream << "<Triangles>" << std::endl;
	vtkCellArray* line = poly_data->GetPolys ();

	if(line)
	{
		long tCount = line->GetNumberOfCells();
		int counttri = 0;
		for (int i = 0; i < tCount*4; i = i+4)
		{
			vtkIdType counta;
			vtkIdType *pts;
			line->GetCell(i, counta, pts);
			
			if(counta!=3)
				continue;
			counttri++;

			xml_stream << "<Triangle IndexNo=\"" << counttri-1 << "\">" << std::endl
				<< "<VertexList>" << pts[0] << " " << pts[1] << " " << pts[2] << "</VertexList>" << std::endl
				<< "<NeighborList>" <<-1<< " " << -1 << " " << -1 << "</NeighborList>" << std::endl
				<< "</Triangle>" << std::endl;
		}
	}
	xml_stream << "</Triangles>" << std::endl;
}

void GeometryXMLWriter::WriteTINBinary(GMPolyData* poly_data,std::ostream& xml_stream)
{
	vtkPoints* points  = poly_data->GetPoints();
	if(points)
	{
		int vCount = points->GetNumberOfPoints();;

		char byteOrder = 0;
		
		int wkbType = 9116;
		int numPoints = vCount;
		char byteOrdercode[sizeof(char)*16];
		int len = Base64Encode(byteOrdercode, (char*)&byteOrder, sizeof(char));		
		xml_stream << byteOrdercode;

		char wkbTypecode[sizeof(int)*4];
		len = Base64Encode(wkbTypecode, (char*)&wkbType, sizeof(int));		
		xml_stream << wkbTypecode;

		char numPointscode[sizeof(int)*4];
		len = Base64Encode(numPointscode, (char*)&numPoints, sizeof(int));		
		xml_stream << numPointscode;

		vtkCellArray* line = poly_data->GetPolys();
		int triCount = 0;
		if(line)
			triCount = line->GetNumberOfCells();

		char numTrianglescode[sizeof(int)*4];
		 len = Base64Encode(numTrianglescode, (char*)&triCount, sizeof(int));		
		xml_stream << numTrianglescode;

		int numTriangleNeighbors = 0;
		char numTriangleNeighborscode[sizeof(int)*4];
		len = Base64Encode(numTriangleNeighborscode, (char*)&numTriangleNeighbors, sizeof(int));		
		xml_stream << numTriangleNeighborscode;

		for(int i = 0; i < vCount; ++i)
		{
			double xx[3];
			points->GetPoint(i,xx);
			char basecode[sizeof(double)*3*4];

			int len1 = Base64Encode(basecode, (char*)xx, sizeof(double)*3);
			xml_stream << basecode;
		}

		int counttri = 0;
		for (int i = 0; i < triCount*4; i = i+4)
		{
			vtkIdType counta;
			vtkIdType *pts;
			line->GetCell(i,counta,pts);

			if(counta!=3)	
				continue;
			counttri++;

			char basecode[sizeof(vtkIdType)*3*4];
			int len1 = Base64Encode(basecode, (char*)pts, sizeof(vtkIdType)*3);
			xml_stream << basecode;
		}
		xml_stream << std::endl;
	}
}


void GeometryXMLWriter::WriteGeoVolumeBinary(GMUnstructuredGrid* grid, std::ostream& xml_stream)
{
	//polydata->GetCell()
	//xmlStream << "<Vertices>" << std::endl;
	vtkPoints* points  = grid->GetPoints();
	//points->GetVoidPointer()
	if(points){
		long i = 0, vCount = points->GetNumberOfPoints();;

		//xmlStream  << "<gml:posList srsDimension=\"3\" count=\"" << vCount << "\">" << std::endl;
		char byteOrder =0;
		int   wkbType = 9120;
		int  numPoints = vCount;
		char byteOrdercode[sizeof(char)*16];
		int len = Base64Encode(byteOrdercode,(char*)&byteOrder,sizeof(char));		
		xml_stream << byteOrdercode;

		char wkbTypecode[sizeof(int)*4];
		len = Base64Encode(wkbTypecode,(char*)&wkbType,sizeof(int));		
		xml_stream << wkbTypecode;

		char numPointscode[sizeof(int)*4];
		len = Base64Encode(numPointscode,(char*)&numPoints,sizeof(int));		
		xml_stream << numPointscode;

		vtkCellArray* line = grid->GetCells ();
		int cellCount = 0;
		if(line)
			cellCount = grid->GetNumberOfCells();

		char numCellscode[sizeof(int)*4];
		len = Base64Encode(numCellscode,(char*)&cellCount,sizeof(int));		
		xml_stream << numCellscode;

		int numCellNeighbors = 0;
		char numCellNeighborscode[sizeof(int)*4];
		len = Base64Encode(numCellNeighborscode,(char*)&numCellNeighbors,sizeof(int));		
		xml_stream << numCellNeighborscode;

		if(grid->GetGeometryType() == 10)
		{
			int cellType=4;
			char cellTypecode[sizeof(int)*16];
			int len = Base64Encode(cellTypecode,(char*)&cellType,sizeof(int));		
			xml_stream<< cellTypecode;
		}

		if(grid->GetGeometryType() == 12)
		{
			int cellType=8;
			char cellTypecode[sizeof(int)*16];
			int len = Base64Encode(cellTypecode,(char*)&cellType,sizeof(int));		
			xml_stream << cellTypecode;
		}

		for(int i = 0; i < vCount; ++i)
		{
			double xx[3];
			points->GetPoint(i,xx);
			char basecode[sizeof(double)*3*4];

			int len1 = Base64Encode(basecode, (char*)xx, sizeof(double)*3);
			xml_stream << basecode;
		}

		int counttri = 0;
		if(grid->GetGeometryType() == 10)
			for (int i = 0; i < cellCount*5; i=i+5)
			{
				vtkIdType counta;
				vtkIdType *pts;
				line->GetCell(i,counta,pts);

				if(counta!=4)		continue;
				counttri++;

				char basecode[sizeof(vtkIdType)*4*4];
				//char 
				int len1 = Base64Encode(basecode,(char*)pts,sizeof(vtkIdType)*4);
				xml_stream << basecode;
			}

			if(grid->GetGeometryType() == 12)
				for (int i = 0; i < cellCount*9; i=i+9)
				{
					vtkIdType counta;
					vtkIdType *pts;
					line->GetCell(i,counta,pts);

					if(counta!=8)		continue;
					counttri++;
			
					char basecode[sizeof(vtkIdType)*8*4];
					//char 
					int len1 = Base64Encode(basecode,(char*)pts,sizeof(vtkIdType)*8);
					xml_stream << basecode;
				}

				xml_stream << std::endl;
	}
}


void GeometryXMLWriter::WriteGeoVolume(GMUnstructuredGrid* grid, std::ostream& xml_stream)
{
	xml_stream << "<Vertices>" << std::endl;
	vtkPoints* points  = grid->GetPoints();

	int vCount = 0;

	if(points)
		vCount = points->GetNumberOfPoints();;
	for (int i = 0; i < vCount; i++)
	{
		double xx[3];
		points->GetPoint(i, xx);

		xml_stream << "<Vertex IndexNo=\"" << i << "\" srsDimension=\"3\">" << xx[0] << " " <<  xx[1] << " " <<  xx[2] << "</Vertex>" << std::endl;
	}
	xml_stream << "</Vertices>" << std::endl;
	xml_stream << "<Polyhedrons>" << std::endl;	
	vtkCellArray* line = grid->GetCells();


	if(grid->GetGeometryType() == 10)
	{


		int tCount = grid->GetNumberOfCells();
		int counttri=0;
		for (int i = 0; i < tCount*5; i = i+5)
		{
			vtkIdType counta;
			vtkIdType *pts;
			line->GetCell(i, counta,pts);

			if(counta!=4)	
				continue;
			counttri++;

			xml_stream << "<Tetrahedron IndexNo=\"" << counttri-1 << "\">" << std::endl
				<< "<VertexList>" << pts[0] << " " << pts[1] << " " << pts[2] <<  " " << pts[3] << "</VertexList>" << std::endl
				<< "<NeighborList>" <<-1<< " " << -1 << " " << -1 << " " << -1 << "</NeighborList>" << std::endl
				<< "</Tetrahedron>" << std::endl;
		}
	}

	if(grid->GetGeometryType() == 12)
	{


		long tCount = grid->GetNumberOfCells();
		int counttri=0;
		for (int i = 0; i < tCount*9; i = i+9)
		{
			vtkIdType counta;
			vtkIdType *pts;
			line->GetCell(i,counta,pts);

			if(counta!=8)	
				continue;
			counttri++;

			xml_stream << "<Cuboid IndexNo=\"" << counttri-1 << "\">" << std::endl
				<< "<VertexList>" << pts[0] << " " << pts[1] << " " << pts[2] <<  " "
				<< pts[3]<<" "<<pts[4] << " " << pts[5] << " " << pts[6] <<  " " << pts[7] <<"</VertexList>" << std::endl				
				<< "</Cuboid>" << std::endl;
		}
	}
	xml_stream << "</Polyhedrons>" << std::endl;
}

void GeometryXMLWriter::WriteStructedPoints(GMStructuredPoints* structed_points, std::ostream& xml_stream)
{
	int dim[3];
	double spacing[3];
	double origin[3];
	structed_points->GetDimensions(dim);
	structed_points->GetSpacing(spacing);
	structed_points->GetOrigin(origin);


	xml_stream << "<gml:RectifiedGrid>" << std::endl;
	xml_stream << "<gml:limits>" << std::endl;
	xml_stream << "<gml:GridEnvelope>" << std::endl;
	xml_stream << "<gml:low>" << 1 << " " << 1 <<" "<< 1 << "</gml:low>" << std::endl;
	xml_stream << "<gml:high>" << dim[0] <<" "<< dim[1] <<" "<< dim[2] << "</gml:high>" << std::endl;

	xml_stream << "</gml:GridEnvelope>" << std::endl;
	xml_stream << "</gml:limits>" << std::endl;

	xml_stream << "<gml:axisLabels>" << "X Y Z" << "</gml:axisLabels>" << std::endl;

	xml_stream << "<gml:origin>" << std::endl;
	xml_stream << "<gml:Point >" << std::endl;
	xml_stream << "<gml:pos>" << origin[0] <<" "<< origin[1] <<" "<< origin[2] << "</gml:pos>" << std::endl;        
	xml_stream << "</gml:Point>" << std::endl ;   
	xml_stream << "</gml:origin>" << std::endl;	;

	xml_stream << "<gml:offsetVector>" << spacing[0] <<" "<< spacing[1] << " " << spacing[2] << "</gml:offsetVector>" << std::endl;
	xml_stream << "</gml:RectifiedGrid>" << std::endl;
}

void GeometryXMLWriter::WriteVertex(GMPolyData* poly_data, std::ostream& xml_stream)
{
	xml_stream << "<Vertices>" << std::endl;
	vtkPoints* points  = poly_data->GetPoints();
	if(points)
	{
		int vCount = points->GetNumberOfPoints();;
		for (int i = 0; i < vCount; i++)
		{
			double xx[3];
			points->GetPoint(i, xx);

			char buf[256];
			sprintf_s(buf,"%lf   %lf  %lf", xx[0], xx[1], xx[2] );
			xml_stream << "<Vertex IndexNo=\"" << i << "\" srsDimension=\"3\">" << buf<< "</Vertex>" << std::endl;
		}
	}
	xml_stream << "</Vertices>" << std::endl;
}
void GeometryXMLWriter::WriteGeoCornerPointGrid(GMCornerPointGrid* poly_data, std::ostream& xml_stream)
{
	int Dimension[3];
	poly_data->GetDimension(Dimension);

	xml_stream << "<Dimension>" ;
	xml_stream<<Dimension[0]<<" "<<Dimension[1]<<" "<<Dimension[2];
	xml_stream << "</Dimension>" << std::endl;
	
	xml_stream << "<Pillars>" << std::endl;
	int PillarCount = poly_data->GetPillarCount();

	for(int i = 0; i < PillarCount; i++)
	{
		double head[3];
		double tail[3];
		poly_data->GetPillar(i,head,tail);
		xml_stream << "<Pillar>" << std::endl;


		xml_stream << "<HeadPos>" ;
		xml_stream<<head[0]<<" "<<head[1]<<" "<<head[2];
		xml_stream << "</HeadPos>" << std::endl;

		xml_stream << "<TailPos>" ;
		xml_stream<<tail[0]<<" "<<tail[1]<<" "<<tail[2];
		xml_stream << "</TailPos>" << std::endl;

		xml_stream << "</Pillar>" << std::endl;
	}	 
	xml_stream << "</Pillars>" << std::endl;


	std::string zz = "ZValue=";
	std::string zzz=poly_data->GetZValueType();

	//有问题
	xml_stream << "<Cells " << zz <<"\""<<zzz<<"\"" << ">"<<std::endl;
	int CellCount = poly_data->GetCellCount();
	std::string cellv = "cellv=";
	for(int i = 0; i < CellCount; i++)
	{
		double Cell[8];
	 
		poly_data->GetCell(i,Cell);
	 


		xml_stream << "<Cell "<<cellv<<"\""<<poly_data->ValidCells[i]<<"\""<<">";
		xml_stream<<Cell[0]<<" "<<Cell[1]<<" "<<Cell[2]<<" "<<Cell[3]<<" ";
		xml_stream<<Cell[4]<<" "<<Cell[5]<<" "<<Cell[6]<<" "<<Cell[7];
		xml_stream << "</Cell>" << std::endl;

	 
 
	}	 
	xml_stream << "</Cells>" << std::endl;

}

void GeometryXMLWriter::WriteGeoCornerPointGridBinary(GMCornerPointGrid* poly_data, std::ostream& xml_stream)
{
	int Dimension[3];
	poly_data->GetDimension(Dimension);

	xml_stream << "<Dimension>" ;
	xml_stream<<Dimension[0]<<" "<<Dimension[1]<<" "<<Dimension[2];
	xml_stream << "</Dimension>" << std::endl;
	xml_stream << "<Pillars>" << std::endl;

	  int numofpillar= poly_data->GetPillarCount();

	if(numofpillar>0)
	{
		
		//xmlStream  << "<gml:posList srsDimension=\"3\" count=\"" << vCount << "\">" << std::endl;
		char byteOrder =0;
		int   wkbType = 9120;
		int  numPillars = numofpillar;
		char byteOrdercode[sizeof(char)*16];
		int len = Base64Encode(byteOrdercode,(char*)&byteOrder,sizeof(char));		
		xml_stream << byteOrdercode;

		char wkbTypecode[sizeof(int)*4];
		len = Base64Encode(wkbTypecode,(char*)&wkbType,sizeof(int));		
		xml_stream << wkbTypecode;

		char numPillarscode[sizeof(int)*4];
		len = Base64Encode(numPillarscode,(char*)&numPillars,sizeof(int));		
		xml_stream << numPillarscode;

		for(int i = 0; i < numPillars; ++i)
		{
			
			double HeadPos[3];
			double TailPos[3];
			poly_data->GetPillar(i,HeadPos,TailPos);
			char basecode[sizeof(double)*3*4];
			char basecode2[sizeof(double)*3*4];

			int len1 = Base64Encode(basecode, (char*)HeadPos, sizeof(double)*3);
			xml_stream << basecode;

			int len2 = Base64Encode(basecode2, (char*)TailPos, sizeof(double)*3);
			xml_stream << basecode2;

		}
	}
	xml_stream <<std::endl;
	xml_stream << "</Pillars>" << std::endl;

	std::string zz = "ZValue=";
	std::string zzz=poly_data->GetZValueType();

	//有问题
	xml_stream << "<Cells " << zz <<"\""<<zzz<<"\"" << ">"<<std::endl;
	int numofcell=poly_data->GetCellCount();
	if(numofcell>0)
	{
		char byteOrder =0;
		int   wkbType = 9120;
		int  numPillars = numofpillar;
		char byteOrdercode[sizeof(char)*16];
		int len = Base64Encode(byteOrdercode,(char*)&byteOrder,sizeof(char));		
		xml_stream << byteOrdercode;

		char wkbTypecode[sizeof(int)*4];
		len = Base64Encode(wkbTypecode,(char*)&wkbType,sizeof(int));		
		xml_stream << wkbTypecode;

		char numCellscode[sizeof(int)*4];
		len = Base64Encode(numCellscode,(char*)&numofcell,sizeof(int));		
		xml_stream << numCellscode;
		for (int i=0;i<numofcell;i++)
		{
			 
			char a=poly_data->ValidCells[i];
			char basecode[sizeof(char)*16];
			int len=Base64Encode(basecode,(char*)&a,sizeof(char));
			xml_stream<<basecode;

			double cell[8];
			poly_data->GetCell(i,cell);
			char basecode2[sizeof(double)*8*4];
			len=Base64Encode(basecode2,(char*)&cell,sizeof(double)*8);
			xml_stream<<basecode2;

		}
		xml_stream << "</Cells>"<<std::endl;

	}

}

 