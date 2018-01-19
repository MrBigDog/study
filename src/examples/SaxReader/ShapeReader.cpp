#include "StdAfx.h"
#include "ShapeReader.h"
#include "gmmlSystem.h"
#include "GMPolyData.h"
#include "vtkPoints.h"
#include "vtkCellArray.h"
//#include "vtkPolyDataNormals.h"
#include "vtkPointData.h"
#include "GMUnstructuredGrid.h"
#include "MappedFeature.h"
#include "GMStructuredPoints.h"
#include "Geo3DProjectReader.h"
#include "XMLBase64.h"
#include "GMCornerPointGrid.h"

#include <sstream>

//GMPolyData *tin = 0;
//GMPolyData *point = 0;
//GMPolyData* line = 0;
//GMUnstructuredGrid* tinUnStructuredGrid = 0;

using gmml::MappedFeature;
using std::stringstream;

vtkObject* ReadShape(void* pData,xmlTextReaderPtr reader,int binary)
{
	
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;

	vtkObject* pVtkObj = NULL;

	while (nRet == 1) 		
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"GeoTin"))
			{
				pVtkObj =  ReadGeoTin(pData,reader,binary);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:Point"))
			{
				pVtkObj =  ReadPoint(reader,binary);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}

			else if (!xmlStrcmp(name,BAD_CAST"gml:MultiPoint"))
			{
				pVtkObj =  ReadMultiPoint(reader,binary);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:LineString"))
			{
				pVtkObj = ReadLineString(reader,binary);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}

			else if (!xmlStrcmp(name,BAD_CAST"gml:MultiCurve"))
			{
				pVtkObj = ReadMultiLineString(reader,binary);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}

			else if (!xmlStrcmp(name,BAD_CAST"GeoVolume"))
			{
				pVtkObj = ReadGeoVolume(reader,binary);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"GeoGrid"))
			{
				pVtkObj = ReadGeoGrid(reader);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}

			else if (!xmlStrcmp(name,BAD_CAST"GeoCornerPointGrid"))
			{
				pVtkObj = ReadGeoCornerPointGrid(reader,binary);
				if (pVtkObj)
				{
					return pVtkObj;
				}
			}

		}

		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Geometry"))
			{
				return 0;
			}
		}


		xmlFree(name);
		nRet = xmlTextReaderRead(reader);
	}
}

vtkObject* ReadGeoTin(void* pData,xmlTextReaderPtr reader,int binary)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;

	GMPolyData *tin = 0;

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoTin"))
			{
				tin = GMPolyData::New();
				tin->SetGeometryType(5);
				
				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					tin->SetID(out);
				}

				xmlChar* binaryvalue = xmlTextReaderGetAttribute(reader,(xmlChar*)"dt:dt");
				if (binaryvalue)
				{
					binary=1;
				}

				else
				{
					binary=0;
				}

				 if(binary)
					{
						xmlChar*szKey=xmlTextReaderReadString(reader);
					char* buffer=(char*)szKey;
			
			
					char pointnum[sizeof(int)*4];
					Base64Decode(pointnum, buffer+13, 8);
					int pointcount=*((int *)(&pointnum));
			
					char celltnum[sizeof(int)*4];
					Base64Decode(celltnum, buffer+21, 8);
					int cellcount=*((int *)(&celltnum));

					vtkPoints*points=vtkPoints::New();

					for(int i=0;i<pointcount;i++)
					{
						char point[sizeof(double)*3*4];
						Base64Decode(point,buffer+37+(i*32),32);
						double a=*((double *)(&point));
						double b=*(((double *)(&point))+1);
						double c=*(((double *)(&point))+2);
						points->InsertNextPoint(a,b,c);
						
					}
			

					vtkCellArray* cellarray=vtkCellArray::New();
					for(int i=0;i<cellcount;i++)
					{
				
						char cell[sizeof(vtkIdType)*3*4];
						Base64Decode(cell,buffer+37+(pointcount*32)+(i*16),16);
						vtkIdType a=*((vtkIdType*)(&cell));
						vtkIdType b=*(((vtkIdType*)(&cell))+1);
						vtkIdType c=*(((vtkIdType*)(&cell))+2);
						cellarray->InsertNextCell(3);
						cellarray->InsertCellPoint(a);
						cellarray->InsertCellPoint(b);
						cellarray->InsertCellPoint(c);
					}
			
					tin->SetPoints(points);
					tin->SetPolys(cellarray);
					return tin;
					/*char byteOrder = 0;
					char byteOrdercode[sizeof(int)];
					Base64Decode(byteOrdercode, buf, sizeof(char)*16);
					int a=*((int *)(&byteOrdercode));
					cout<<a<<endl;*/
					}
		    }

		


			else if (!xmlStrcmp(name,BAD_CAST"Vertices"))
			{
				ReadVertices(tin,reader);
			}
			else if (!xmlStrcmp(name,BAD_CAST"Triangles"))
			{
				vtkCellArray *line = vtkCellArray::New();
				tin->SetPolys(line);
			}
			else if (!xmlStrcmp(name,BAD_CAST"Triangle"))
			{
				ReadTriangles(tin,reader);
			}
		}
		else if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoTin"))
			{
				//((MappedFeature*)pData)->SetShape(tin);

				return tin;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}
}

//可进一步减少
int ReadVertices(void* pData,xmlTextReaderPtr reader)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	while (nRet == 1) 		
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Vertex"))
			{
				char* pIndexNo = ReadAttrAndConvert(reader,"IndexNo");
				if (pIndexNo)
				{
					int vid = atoi((char*)pIndexNo);

					nRet = xmlTextReaderRead(reader);

					if((vid >= 16620 && vid <= 16625) || vid == 14198 || (vid >= 12464 && vid <= 12472))
					{
						value = xmlTextReaderValue(reader);
						if (value)
						{
							double x = 0, y = 0, z = 0;
							char buf[256] = {'\0'};
							int cc = 0;
							for(int ii = 0; ii < strlen((char*)value); ii++)
							{
								if(value[ii] != ',')
									buf[cc++] = value[ii];
							}
							sscanf(buf,"%lf %lf %lf",&x,&y,&z);

							vtkPoints* points  = ((GMPolyData*)pData)->GetPoints();

							if(!points)
							{ 
								points = vtkPoints::New();
								points->SetDataTypeToDouble();
								((GMPolyData*)pData)->SetPoints(points);
							}

							points->InsertPoint(vid,x,y,z);

							xmlFree(value);
						}							
					}
					else
					{
						value = xmlTextReaderReadString(reader);
						if (value)
						{
							double x = 0, y = 0, z = 0;
							char buf[256] = {'\0'};
							int cc = 0;
							for(int ii = 0; ii < strlen((char*)value); ii++)
							{
								if(value[ii] != ',')
									buf[cc++] = value[ii];
							}
							sscanf(buf,"%lf %lf %lf",&x,&y,&z);

							vtkPoints* points  = ((GMPolyData*)pData)->GetPoints();

							if(!points)
							{
								points = vtkPoints::New();
								points->SetDataTypeToDouble();
								((GMPolyData*)pData)->SetPoints(points);
							}

							points->InsertPoint(vid,x,y,z);

							xmlFree(value);
						}

					}
				}			
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Vertices"))
			{
				return 0;
			}

		}
		xmlFree(name);
		nRet = xmlTextReaderRead(reader);
	}
							
}


int ReadTriangles(void* pData,xmlTextReaderPtr reader)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	while (nRet == 1) 		
	{
		name = xmlTextReaderName(reader);

		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Triangle"))
			{
				int nTriID = -1;

				char* pIndexNo = ReadAttrAndConvert(reader,"IndexNo");
				if (pIndexNo)
				{
					nTriID = atoi((char*)pIndexNo);					
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"VertexList"))
			{
				xmlChar* vertex_list = NULL,* value = NULL;
				int x = -1,y = -1,z = -1;

				vertex_list = xmlTextReaderReadString(reader);					
				if (vertex_list)
				{
					sscanf((char*)vertex_list,"%d %d %d",&x,&y,&z);

					vtkCellArray* line = ((GMPolyData*)pData)->GetPolys();

					line->InsertNextCell(3);
					line->InsertCellPoint(x);
					line->InsertCellPoint(y);
					line->InsertCellPoint(z);

					xmlFree(vertex_list);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"NeighborList"))
			{
				xmlChar* neighbor_List = NULL,* value = NULL;
				int t1 = -1,t2 = -1,t3 = -1;

				neighbor_List = xmlTextReaderReadString(reader);
				if (neighbor_List)
				{
					sscanf((char*)neighbor_List,"%d %d %d",&t1,&t2,&t3);  

					xmlFree(neighbor_List);
				}
			}
		
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Triangle"))
			{
				return 0;
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadCells(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL,* prev_name = NULL;
	int nRet = 1;
	
	int TriID = -9999;
	int x = -1,y = -1,z = -1,t=-1;
	int t1=-1,t2=-1,t3 = -1,t4=-1;

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if ((!xmlStrcmp(name, BAD_CAST"Tetrahedron"))) 
			{
				((GMUnstructuredGrid*)pData)->SetGeometryType(10);
				
				char* out = ReadAttrAndConvert(reader,"IndexNo");
				if (out)
				{
					TriID = atoi(out);
				}
			} 
			else if (!xmlStrcmp(name,BAD_CAST"VertexList")&&
				!xmlStrcmp(prev_name,BAD_CAST"Tetrahedron"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					sscanf((char*)value,"%d %d %d %d",&x,&y,&z,&t);  
				}

				xmlFree(value);
			}
			else if (!xmlStrcmp(name,BAD_CAST"NeighborList"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					sscanf((char*)value,"%d %d %d %d",&t1,&t2,&t3,&t4); 
				}

				vtkIdType cell[4] ={x,y,z,t};
				((GMUnstructuredGrid*)pData)->InsertNextCell(10,4,cell); 

				xmlFree(value);
			}
			else if (!xmlStrcmp(name,BAD_CAST"Cuboid"))
			{
				((GMUnstructuredGrid*)pData)->SetGeometryType(12);
				char* out = ReadAttrAndConvert(reader,"IndexNo");
				if (out)
				{
					TriID =atoi(out);
				}				
			}
			else if (!xmlStrcmp(name, (const xmlChar *)"VertexList")&&
				!xmlStrcmp(prev_name,BAD_CAST"Cuboid"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					vtkIdType cell[8]; 

					sscanf((char*)value,"%d %d %d %d %d %d %d %d",&cell[0],&cell[1],&cell[2],&cell[3],&cell[4],&cell[5],&cell[6],&cell[7]);  
					
					((GMUnstructuredGrid*)pData)->InsertNextCell(12,8,cell);  
				}
				xmlFree(value);
				
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"Polyhedrons"))
			{
				return 0;
			}
		}

		if (prev_name)
		{
			xmlFree(prev_name);
		}
		
		if (name)
		{
			prev_name = name;
		}

		nRet = xmlTextReaderRead(reader);
	}
}


vtkObject* ReadMultiPoint(xmlTextReaderPtr reader,int binary)
{
	int numofpoint=0;
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	GMPolyData *point = 0;
	vtkPoints *points=vtkPoints::New();
	vtkCellArray* lines=vtkCellArray::New();
	while (nRet == 1) 		
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if ((!xmlStrcmp(name,BAD_CAST"gml:MultiPoint"))) 
			{
				
				char* out = (char*)xmlTextReaderGetAttribute(reader,BAD_CAST"gml:id");
				if (out)
				{
					point = GMPolyData::New();
					
					point->SetGeometryType(2);		
					point->SetID(out);
				}

			}

			else if (!xmlStrcmp(name,BAD_CAST"gml:pos"))
			{
				char* out = (char*)xmlTextReaderGetAttribute(reader,BAD_CAST"srsDimension");
				if (out)
				{
					int point_cnt = atoi(out);
					xmlChar* value = xmlTextReaderReadString(reader);
					if (value&&point_cnt>=2)
					{
						double x,y,z=0;

						char buf[256];
						int cc = 0;
						for(int ii = 0; ii < strlen((char*)value); ii++)
						{
							if(value[ii] != ',')
								buf[cc++] = value[ii];
						}
						sscanf(buf,"%lf %lf %lf",&x,&y,&z);

						
						points->InsertNextPoint(x,y,z);		
						numofpoint++;
					}

				

					
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:MultiPoint"))
			{
				for(int i = 0; i < numofpoint; i++)
				{
					lines->InsertNextCell(1);
					lines->InsertCellPoint(i);					 
				}

				point->SetPoints(points);
				point->SetVerts(lines);

				return point;
			}
		}

		xmlFree(name);
		nRet = xmlTextReaderRead(reader);
	}
}


vtkObject* ReadPoint(xmlTextReaderPtr reader,int binary)
{
	int nRet = 1;
	xmlChar* name = NULL,*value = NULL;
	GMPolyData *point = 0;

	while (nRet == 1) 		
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if ((!xmlStrcmp(name,BAD_CAST"gml:Point"))) 
			{
				char* out = (char*)xmlTextReaderGetAttribute(reader,BAD_CAST"gml:id");
				if (out)
				{
					point = GMPolyData::New();

					point->SetGeometryType(1);		
					point->SetID(out);
				}


				xmlChar* binaryvalue = xmlTextReaderGetAttribute(reader,(xmlChar*)"dt:dt");
				if (binaryvalue)
				{
					binary=1;
				}

				else
				{
					binary=0;
				}

				if(binary)
				{
					xmlChar*szKey=xmlTextReaderReadString(reader);
					char* buffer=(char*)szKey;


					char pointnum[sizeof(int)*4];
					Base64Decode(pointnum, buffer+13, 8);
					int pointcount=*((int *)(&pointnum));


					vtkPoints*points=vtkPoints::New();
					vtkCellArray*cellarray=vtkCellArray::New();
					for(int i=0;i<pointcount;i++)
					{
						char pointss[sizeof(double)*3*4];
						Base64Decode(pointss,buffer+21+(i*32),32);
						double a=*((double *)(&pointss));
						double b=*(((double *)(&pointss))+1);
						double c=*(((double *)(&pointss))+2);
						points->InsertPoint(i,a,b,c);
						cellarray->InsertNextCell(1);
						cellarray->InsertCellPoint(i);

					}

					point->SetPoints(points);
					point->SetVerts(cellarray);

					return point;

				}
			}

			else if (!xmlStrcmp(name,BAD_CAST"gml:pos"))
			{
				char* out = (char*)xmlTextReaderGetAttribute(reader,BAD_CAST"srsDimension");
				if (out)
				{
					int point_cnt = atoi(out);
					xmlChar* value = xmlTextReaderReadString(reader);
					if (value&&point_cnt>=2)
					{
						double x,y,z=0;

						char buf[256];
						int cc = 0;
						for(int ii = 0; ii < strlen((char*)value); ii++)
						{
							if(value[ii] != ',')
								buf[cc++] = value[ii];
						}
						sscanf(buf,"%lf %lf %lf",&x,&y,&z);

						vtkPoints* points  = point->GetPoints();

						if(!points)
						{  
							points = vtkPoints::New();
							points->SetDataTypeToDouble();
							point->SetPoints(points);

							vtkCellArray*	 lines = vtkCellArray::New();
							point->SetVerts(lines);
						}
						points->InsertPoint(0,x,y,z);		

					}

					vtkCellArray* lines  = point->GetVerts();

					for(int i = 0; i < 1; i++)
					{
						lines->InsertNextCell(1);
						lines->InsertCellPoint(0);					 
					}
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:Point"))
			{
				return point;
			}
		}

		xmlFree(name);
		nRet = xmlTextReaderRead(reader);
	}
}


vtkObject* ReadMultiLineString(xmlTextReaderPtr reader,int binary)
{
	xmlChar* name;
	int nRet = 1;
	GMPolyData* line = 0;
	int numofpoint=0;
	vtkPoints *points=vtkPoints::New();
	vtkCellArray* lines=vtkCellArray::New();
	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);		
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:MultiCurve"))
			{
				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					line = GMPolyData::New();
					line->SetGeometryType(4);
					line->SetID(out);
				}
			}
			else if (!xmlStrcmp(name, (const xmlChar *)"gml:posList"))			
			{
				int point_cnt  = 0;

				char*out = ReadAttrAndConvert(reader,"count");
				if (out)
				{
					point_cnt = atoi(out);
					if(point_cnt>0)
					{

						xmlChar*value = xmlTextReaderReadString(reader);      
						std::string pointlst((char*)value);


						std::istringstream istr((char*)value);
						for(int i =0 ; i < point_cnt; i++)
						{
							double x,y,z ;
							istr>>x; istr>>y; istr>>z; 
							//datapointer[i] =a;

							
							points->InsertNextPoint(x,y,z);	
						}

						lines->InsertNextCell(point_cnt);
						for(int i = 0; i < point_cnt; i++)
						{
							lines->InsertCellPoint(i+numofpoint);
						}
						numofpoint=numofpoint+point_cnt;
					}
					else
					{

						std::string error = "非法数据：线的点个数为0" + line->GetID();
						gmmlDisplayErrorandWarningText(error.c_str());

					}
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:MultiCurve"))
			{
				line->SetPoints(points);
				line->SetLines(lines);

				vtkPoints* points  = line->GetPoints();
				if(!points || points->GetNumberOfPoints() < 1)
				{
					std::string error = "非法数据：线的点个数为0" + line->GetID();
					gmmlDisplayErrorandWarningText(error.c_str());
				}
				else
					return line;
				
					
			}

		}

		nRet = xmlTextReaderRead(reader);
	}
}


vtkObject* ReadLineString(xmlTextReaderPtr reader,int binary)
{
	xmlChar* name;
	int nRet = 1;
	GMPolyData* line = 0;


	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);		
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:LineString"))
			{
				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					line = GMPolyData::New();
					line->SetGeometryType(3);
					vtkPoints* points  = line->GetPoints();

					line->SetID(out);
				}


				xmlChar* binaryvalue = xmlTextReaderGetAttribute(reader,(xmlChar*)"dt:dt");
				if (binaryvalue)
				{
					binary=1;
				}

				else
				{
					binary=0;
				}

				if(binary)
				{
					xmlChar*szKey=xmlTextReaderReadString(reader);
					char* buffer=(char*)szKey;


					char pointnum[sizeof(int)*4];
					Base64Decode(pointnum, buffer+13, 8);
					int pointcount=*((int *)(&pointnum));


					vtkPoints*points=vtkPoints::New();
					vtkCellArray*cellarray=vtkCellArray::New();

					for(int i=0;i<pointcount;i++)
					{
						char pointss[sizeof(double)*3*4];
						Base64Decode(pointss,buffer+21+(i*32),32);
						double a=*((double *)(&pointss));
						double b=*(((double *)(&pointss))+1);
						double c=*(((double *)(&pointss))+2);
						points->InsertPoint(i,a,b,c);

					}

					cellarray->InsertNextCell(pointcount);
					for(int i=0;i<pointcount;i++)
					{
						cellarray->InsertCellPoint(i);
					}

					line->SetPoints(points);
					line->SetLines(cellarray);

					return line;

				}
			}
			else if (!xmlStrcmp(name, (const xmlChar *)"gml:posList"))			
			{
				int point_cnt  = 0;

				char*out = ReadAttrAndConvert(reader,"count");
				if (out)
				{
					point_cnt = atoi(out);
					if(point_cnt>0)
					{

						xmlChar*value = xmlTextReaderReadString(reader);      
						std::string pointlst((char*)value);


						std::istringstream istr((char*)value);
						for(int i =0 ; i < point_cnt; i++)
						{
							double x,y,z ;
							istr>>x; istr>>y; istr>>z; 
							//datapointer[i] =a;

							vtkPoints* points  = line->GetPoints();
							if(!points)
							{ 
								points = vtkPoints::New();
								points->SetDataTypeToDouble();
								line->SetPoints(points);

								vtkCellArray*  lines = vtkCellArray::New();
								line->SetLines(lines);
							}
							points->InsertPoint(i,x,y,z);	
						}

						vtkCellArray* lines  = line->GetLines();
						for(int i = 0; i < point_cnt-1; i++)
						{
							lines->InsertNextCell(2);
							lines->InsertCellPoint(i);
							lines->InsertCellPoint(i+1);
						}
					}
					else
					{
				
						std::string error = "非法数据：线的点个数为0" + line->GetID();
						gmmlDisplayErrorandWarningText(error.c_str());
				
					}
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:LineString"))
			{
				vtkPoints* points  = line->GetPoints();
				if(!points || points->GetNumberOfPoints() < 1)
				{
					std::string error = "非法数据：线的点个数为0" + line->GetID();
					gmmlDisplayErrorandWarningText(error.c_str());
				}
				else
					return line;
			}
			
		}

		nRet = xmlTextReaderRead(reader);
	}
}

vtkObject* ReadGeoVolume(xmlTextReaderPtr reader,int binary)
{
	xmlChar* name;
	int nRet = 1;

	GMUnstructuredGrid* tinUnStructuredGrid = NULL;

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoVolume"))
			{
				tinUnStructuredGrid = GMUnstructuredGrid::New();
				
				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					tinUnStructuredGrid->SetID(out);
				}

				xmlChar* binaryvalue = xmlTextReaderGetAttribute(reader,(xmlChar*)"dt:dt");
				if (binaryvalue)
				{
					binary=1;
				}

				else
				{
					binary=0;
				}

				if(binary)
				{

					xmlChar*szKey=xmlTextReaderReadString(reader);
					char* buffer=(char*)szKey;



					char cellTypecode[sizeof(int)*16];
					Base64Decode(cellTypecode,buffer+37,8);	
					int celltype=*((int *)(&cellTypecode));

					if(celltype==4)
					{
						tinUnStructuredGrid->SetGeometryType(10);
						char pointnum[sizeof(int)*4];
						Base64Decode(pointnum, buffer+13, 8);
						int pointcount=*((int *)(&pointnum));

						char celltnum[sizeof(int)*4];
						Base64Decode(celltnum, buffer+21, 8);
						int cellcount=*((int *)(&celltnum));

						vtkPoints*points=vtkPoints::New();
						points->SetDataTypeToDouble();

						for(int i=0;i<pointcount;i++)
						{
							char point[sizeof(double)*3*4];
							Base64Decode(point,buffer+45+(i*32),32);
							double a=*((double *)(&point));
							double b=*(((double *)(&point))+1);
							double c=*(((double *)(&point))+2);
							points->InsertNextPoint(a,b,c);

						}


						tinUnStructuredGrid->SetPoints(points);
						for(int i=0;i<cellcount;i++)
						{

							char cell[sizeof(vtkIdType)*4*4];
							Base64Decode(cell,buffer+45+(pointcount*32)+(i*24),24);
							vtkIdType a=*((vtkIdType*)(&cell));
							vtkIdType b=*(((vtkIdType*)(&cell))+1);
							vtkIdType c=*(((vtkIdType*)(&cell))+2);
							vtkIdType d=*(((vtkIdType*)(&cell))+3);
							vtkIdType ncell[4] ={a,b,c,d};
							tinUnStructuredGrid->InsertNextCell(10,4,ncell);
						}

						// tin->SetPoints(points);
						// tin->SetPolys(cellarray);
					}

					if(celltype==8)
					{
						tinUnStructuredGrid->SetGeometryType(12);
						char pointnum[sizeof(int)*4];
						Base64Decode(pointnum, buffer+13, 8);
						int pointcount=*((int *)(&pointnum));

						char celltnum[sizeof(int)*4];
						Base64Decode(celltnum, buffer+21, 8);
						int cellcount=*((int *)(&celltnum));

						vtkPoints*points=vtkPoints::New();
						points->SetDataTypeToDouble();

						for(int i=0;i<pointcount;i++)
						{
							char point[sizeof(double)*3*4];
							Base64Decode(point,buffer+45+(i*32),32);
							double a=*((double *)(&point));
							double b=*(((double *)(&point))+1);
							double c=*(((double *)(&point))+2);
							points->InsertNextPoint(a,b,c);

						}


						tinUnStructuredGrid->SetPoints(points);
						for(int i=0;i<cellcount;i++)
						{

							char cell[sizeof(vtkIdType)*8*4];
							Base64Decode(cell,buffer+45+(pointcount*32)+(i*44),44);
							vtkIdType a=*((vtkIdType*)(&cell));
							vtkIdType b=*(((vtkIdType*)(&cell))+1);
							vtkIdType c=*(((vtkIdType*)(&cell))+2);
							vtkIdType d=*(((vtkIdType*)(&cell))+3);
							vtkIdType e=*(((vtkIdType*)(&cell))+4);
							vtkIdType f=*(((vtkIdType*)(&cell))+5);
							vtkIdType g=*(((vtkIdType*)(&cell))+6);
							vtkIdType h=*(((vtkIdType*)(&cell))+7);

							vtkIdType ncell[8] ={a,b,c,d,e,f,g,h};
							tinUnStructuredGrid->InsertNextCell(12,8,ncell);
						}
					}

					return tinUnStructuredGrid;
				}

			}
			else if (!xmlStrcmp(name,BAD_CAST"Vertices"))
			{
				ReadVertices(tinUnStructuredGrid,reader);
			}
			else if (!xmlStrcmp(name,BAD_CAST"Polyhedrons"))
			{
				ReadCells(tinUnStructuredGrid,reader);
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"GeoVolume"))
			{
				return tinUnStructuredGrid;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}
}

vtkObject* ReadGeoGrid(xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;
	GMStructuredPoints* tin = NULL;

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoGrid"))
			{
				tin = GMStructuredPoints::New();

				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					tin->SetID(out);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:RectifiedGrid"))
			{
				ReadRectifiedGrid(tin,reader);
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"GeoGrid"))
			{
				return tin;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}
}

vtkObject* ReadGeoCornerPointGrid(xmlTextReaderPtr reader,int binary)
{
	xmlChar* name = NULL;
	int nRet = 1;
	GMCornerPointGrid* tin = NULL;

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"GeoCornerPointGrid"))
			{
				tin = GMCornerPointGrid::New();

				char* out = ReadAttrAndConvert(reader,"gml:id");
				if (out)
				{
					tin->SetID(out);
				}


				xmlChar* binaryvalue = xmlTextReaderGetAttribute(reader,(xmlChar*)"dt:dt");
				if (binaryvalue)
				{
					binary=1;
				}

				else
				{
					binary=0;
				}
			}

			else if (!xmlStrcmp(name,BAD_CAST"Dimension"))
			{
				double x=0,y=0,z=0;
			    int dimension[3];
				char* out = ReadValueAndConvert(reader);

				sscanf((char*)out,"%lf %lf %lf",&x,&y,&z); 
				dimension[0]=x;
				dimension[1]=y;
				dimension[2]=z;
				tin->SetDimension(dimension);
			}

			else if (!xmlStrcmp(name,BAD_CAST"Pillars"))
			{
				if(binary)
				{
					xmlChar* sZkey=xmlTextReaderReadString(reader);
					char* buffer=(char*)sZkey;
					char numofpillar[sizeof(int)*4];
					Base64Decode(numofpillar,buffer+13,8);
					int pillarcount=*((int *)(&numofpillar));

					for(int i=0;i<pillarcount;i++)
					{
						char head[sizeof(double)*3*4];
						Base64Decode(head,buffer+21+2*i*32,32);

						double headpos[3];
						headpos[0]=*((double*)(&head));
						headpos[1]=*(((double*)(&head))+1);
						headpos[2]=*(((double*)(&head))+2);

						char tail[sizeof(double)*3*4];
						Base64Decode(tail,buffer+53+2*i*32,32);

						double tailpos[3];
						tailpos[0]=*((double*)(&tail));
						tailpos[1]=*(((double*)(&tail))+1);
						tailpos[2]=*(((double*)(&tail))+2);

						tin->AddPillar(headpos,tailpos);
					}


				}
				else
				{
					ReadPillars(tin,reader);
				}
				
			}

			else if (!xmlStrcmp(name,BAD_CAST"Cells"))
			{
				char* attrout = ReadAttrAndConvert(reader,"ZValue");
				if (attrout)
				{
					std::string zvaluetype=(std::string)(attrout);
					tin->SetZValueType(zvaluetype);
				}

				if(binary)
				{
					xmlChar* szKey=xmlTextReaderReadString(reader);
					char* buffer=(char*)szKey;

					char cellnum[sizeof(int)*4];
					Base64Decode(cellnum, buffer+13, 8);
					int cellcount=*((int *)(&cellnum));

					for(int i=0;i<cellcount;i++)
					{
						char cellv;
						char valid[sizeof(char)*16];
						Base64Decode(valid,buffer+21+i*4+i*88,4);
						cellv=*((char*)(&valid));

						double cell[8];
						char Cell[sizeof(double)*8*4];
						Base64Decode(Cell,buffer+25+i*88+i*4,88);
						cell[0]=*(((double*)(&Cell))+0);
						cell[1]=*(((double*)(&Cell))+1);
						cell[2]=*(((double*)(&Cell))+2);
						cell[3]=*(((double*)(&Cell))+3);
						cell[4]=*(((double*)(&Cell))+4);
						cell[5]=*(((double*)(&Cell))+5);
						cell[6]=*(((double*)(&Cell))+6);
						cell[7]=*(((double*)(&Cell))+7);

						tin->AddCell(cell,cellv);

					}
				}

				else
				{
					ReadCCells(tin,reader);
				}
				
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"GeoCornerPointGrid"))
			{
				return tin;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}
}


int ReadPillars(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;
	GMCornerPointGrid* tin = (GMCornerPointGrid*)pData;
	if (!tin)
	{
		return -1;
	}

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			double xyz1[3];
			double xyz2[3];
			if (!xmlStrcmp(name,BAD_CAST"HeadPos"))
			{
				char* out = ReadValueAndConvert(reader);
				sscanf((char*)out,"%lf %lf %lf",&xyz1[0],&xyz1[1],&xyz1[2]);
			}
			else if (!xmlStrcmp(name,BAD_CAST"TailPos"))
			{
				char* out = ReadValueAndConvert(reader);
				sscanf((char*)out,"%lf %lf %lf",&xyz2[0],&xyz2[1],&xyz2[2]);
				tin->AddPillar(xyz1,xyz2);
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"Pillars"))
			{
				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}
}


int ReadCCells(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;
	GMCornerPointGrid* tin = (GMCornerPointGrid*)pData;
	if (!tin)
	{
		return -1;
	}

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			
			if (!xmlStrcmp(name,BAD_CAST"Cell"))
			{ 
				xmlChar* attrout = xmlTextReaderGetAttribute(reader,(xmlChar*)"cellv");
				if (attrout)
				{   
					char valid=*((char*)attrout);
					double value[8]={0,0,0,0,0,0,0,0};
					
					
					xmlChar* valueout = xmlTextReaderReadString(reader);

					char* buffer=(char*)valueout;
					sscanf(buffer,"%lf %lf %lf %lf %lf %lf %lf %lf",&value[0],&value[1],&value[2],&value[3],&value[4],&value[5],&value[6],&value[7]);

					tin->AddCell(value,valid);
				}
				
				
			}
			
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"Cells"))
			{
				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}
}

//注意可能有错误，检查xml
int ReadRectifiedGrid(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;
	GMStructuredPoints* tin = (GMStructuredPoints*)pData;
	if (!tin)
	{
		return -1;
	}

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			
			 if (!xmlStrcmp(name,BAD_CAST"gml:GridEnvelope"))
			{
				ReadGridEnvelope(tin,reader);
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:Point"))
			{
				ReadPos(tin,reader);
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:offsetVector"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					double x,y,z;
					sscanf((char*)out,"%lf %lf %lf",&x,&y,&z);

					tin->SetSpacing( x,y,z);
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"gml:RectifiedGrid"))
			{
				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}
}

int ReadGridEnvelope(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;
	double lx=0,ly=0,lz=0;
	double hx=1,hy=1,hz=1;


	GMStructuredPoints* tin = (GMStructuredPoints*)pData;
	if (!tin)
	{
		return -1;
	}

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:low"))
			{
				xmlChar* out = xmlTextReaderReadString(reader);
				if (out)
				{
					double x,y,z;
					sscanf((char*)out,"%lf %lf %lf",&lx,&ly,&lz);
				}
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:high"))
			{
				xmlChar* out = xmlTextReaderReadString(reader);
				if (out)
				{
					double x,y,z;
					sscanf((char*)out,"%lf %lf %lf",&hx,&hy,&hz);
				}
			}
			/*else if (!xmlStrcmp(name,BAD_CAST"gml:pos"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				double x,y,z;
				sscanf((char*)value,"%lf %lf %lf",&x,&y,&z); 	
				tin->SetOrigin( x,y,z);
				
				xmlFree(value);
			}
			else if (!xmlStrcmp(name,BAD_CAST"gml:offsetVector"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				double x,y,z;
				sscanf((char*)value,"%lf %lf %lf",&x,&y,&z);				
				tin->SetSpacing( x,y,z);

				xmlFree(value);
			}*/
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"gml:GridEnvelope"))
			{
				tin->SetDimensions(hx-lx+1,hy-ly+1,hz-lz+1);				
				
				return 0;
			}


		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadPos(void* pData,xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;

	GMStructuredPoints* tin = (GMStructuredPoints*)pData;
	if (!tin)
	{
		return -1;
	}

	while(nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if(!xmlStrcmp(name,BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name,BAD_CAST"gml:pos"))
			{
				char* out = ReadValueAndConvert(reader);
				if (out)
				{
					double x,y,z;
					sscanf((char*)out,"%lf %lf %lf",&x,&y,&z); 	
					tin->SetOrigin( x,y,z);
				}
			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if(!xmlStrcmp(name,BAD_CAST"gml:Point"))
			{
				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}
}

