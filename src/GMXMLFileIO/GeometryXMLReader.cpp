#include "GeometryXMLReader.h"

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
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"

#include "vtkCharArray.h"
#include "vtkLongArray.h"
#include <sstream>
#include <string>
#include <vector>
#include "gmmlSystem.h"
#include "XMLBase64.h"

int GeometryXMLReader::BinaryExport;
GeometryXMLReader::GeometryXMLReader(void)
{
	ShapeObject = 0;
}


GeometryXMLReader::~GeometryXMLReader(void)
{
}

vtkObject* GeometryXMLReader::ReadShape(xmlNode* geometryNode)
{
	vtkObject* data = ReadPolyDataTIN(geometryNode);
	if(data)
		return data;

	data = ReadPolyDataTEN(geometryNode);
	if(data)
		return data;

	data = ReadPolyDataPoints(geometryNode);
	if(data)
		return data;

	data = ReadPolyDataLine(geometryNode);
	if(data)
		return data;

	data = ReadGeoVolume(geometryNode);
	if(data)
		return data;

	data = ReadGeoGrid(geometryNode);
	if(data)
		return data;

    data = ReadGeoCornerPointGrid(geometryNode);
	if(data)
		return data;
	return 0;
}

vtkObject* GeometryXMLReader::ReadGeoGrid(xmlNode* geometryNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量

	GMStructuredPoints *tin = 0;
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"GeoGrid"))) 
	{
		tin = GMStructuredPoints::New();
		//
		//查找带有属性attribute的节点
		if (xmlHasProp(curNode,BAD_CAST "id"))
		{
			propNodePtr = curNode;
			xmlAttrPtr attrPtr = propNodePtr->properties;
			if (!xmlStrcmp(attrPtr->name, BAD_CAST "id"))
			{			 
				xmlChar* attr_str = xmlGetProp(propNodePtr, BAD_CAST "id");
				char* out = NULL;
				if(std::string(mXMLCoding) == "UTF-8")
					out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
				std::string sr = "";
				if (out)
					sr=std::string((char*)out);
				else
					sr=std::string((char*)attr_str);

				tin->SetID(sr);
				xmlFree(attr_str);
			}
		}

		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode != NULL && tin) 
		{				
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"RectifiedGrid"))) 
			{
				ReadRectifiedGrid(tin,curChildNode);
			}

			curChildNode = curChildNode->next; 
		}		  
	}   


	return tin;
}


vtkObject* GeometryXMLReader::ReadGeoCornerPointGrid(xmlNode* geometryNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量

	GMCornerPointGrid *tin = 0;
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"GeoCornerPointGrid"))) 
	{
		tin = GMCornerPointGrid::New();
		//
		//查找带有属性attribute的节点
		if (xmlHasProp(curNode,BAD_CAST "id"))
		{
			propNodePtr = curNode;
			xmlAttrPtr attrPtr = propNodePtr->properties;
			if (!xmlStrcmp(attrPtr->name, BAD_CAST "id"))
			{			 
				xmlChar* attr_str = xmlGetProp(propNodePtr, BAD_CAST "id");
				char* out = NULL;
				if(std::string(mXMLCoding) == "UTF-8")
					out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
				std::string sr = "";
				if (out)
					sr=std::string((char*)out);
				else
					sr=std::string((char*)attr_str);

				tin->SetID(sr);
				xmlFree(attr_str);
			}

		}

		
		if (xmlHasProp(curNode,BAD_CAST "dt"))
		{


			BinaryExport=1;
		}

		else
		{
			BinaryExport=0;
		}
		
		

		if(BinaryExport)
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL && tin) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Dimension"))) 
				{
					double x=0,y=0,z=0;
					int dimension[3];
					szKey = xmlNodeGetContent(curChildNode);

					sscanf((char*)szKey,"%lf %lf %lf",&x,&y,&z); 
					dimension[0]=x;
					dimension[1]=y;
					dimension[2]=z;
					tin->SetDimension(dimension);

					xmlFree(szKey);
				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Pillars"))) 
				{
					xmlChar* szKey=xmlNodeGetContent(curChildNode);
					char* buffer=(char*)szKey;


					char pillarnum[sizeof(int)*4];
					Base64Decode(pillarnum, buffer+13, 8);
					int pillarcount=*((int *)(&pillarnum));

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

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Cells"))) 
				{
					if (xmlHasProp(curNode,BAD_CAST "Zvalue"))
					{
						propNodePtr = curNode;
						xmlAttrPtr attrPtr = propNodePtr->properties;
						if (!xmlStrcmp(attrPtr->name, BAD_CAST "Zvalue"))
						{			 
							xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "Zvalue");
							tin->SetZValueType((char*)szAttr);
							xmlFree(szAttr);
						}
					}
					xmlChar* szKey=xmlNodeGetContent(curChildNode);
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

				curChildNode = curChildNode->next; 
			}	


		}
		else
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL && tin) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Dimension"))) 
				{
					double x=0,y=0,z=0;
					int dimension[3];
					szKey = xmlNodeGetContent(curChildNode);

					sscanf((char*)szKey,"%lf %lf %lf",&x,&y,&z); 
					dimension[0]=x;
					dimension[1]=y;
					dimension[2]=z;
					tin->SetDimension(dimension);

					xmlFree(szKey);
				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Pillars"))) 
				{
					ReadPillars(tin,curChildNode);
				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Cells"))) 
				{
					ReadCCells(tin,curChildNode);
				}

				curChildNode = curChildNode->next; 
			}	
		}

	}   


	return tin;
}

bool GeometryXMLReader::ReadPillars(GMCornerPointGrid* data,xmlNode* geometryNode)
{
	double headpos[3];
	double tailpos[3];
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量
	if(data)
	{
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Pillars"))) 
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			{
				while (curChildNode!=NULL)
				{
					if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Pillar"))) 
					{
						xmlNodePtr originNode = curChildNode->xmlChildrenNode;
						while(originNode != NULL) 
						{
							
							if ((!xmlStrcmp(originNode->name, (const xmlChar *)"HeadPos"))) 
							{
								
								szKey = xmlNodeGetContent(originNode);
								sscanf((char*)szKey,"%lf %lf %lf",&headpos[0],&headpos[1],&headpos[2]); 
							}
							if ((!xmlStrcmp(originNode->name, (const xmlChar *)"TailPos"))) 
							{
								
								szKey = xmlNodeGetContent(originNode);
								sscanf((char*)szKey,"%lf %lf %lf",&tailpos[0],&tailpos[1],&tailpos[2]); 
								data->AddPillar(headpos,tailpos);
							}

							originNode=originNode->next;
						}
					}

					curChildNode=curChildNode->next;
				}
			}
		}
	}
	return true;
}

bool GeometryXMLReader::ReadCCells(GMCornerPointGrid* data,xmlNode* geometryNode)
{
	double headpos[3];
	double tailpos[3];
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量
	if(data)
	{
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Cells"))) 
		{
			if (xmlHasProp(curNode,BAD_CAST "Zvalue"))
			{
				propNodePtr = curNode;
				xmlAttrPtr attrPtr = propNodePtr->properties;
				if (!xmlStrcmp(attrPtr->name, BAD_CAST "Zvalue"))
				{			 
					xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "Zvalue");
					data->SetZValueType((char*)szAttr);
					xmlFree(szAttr);
				}
			}
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			{
				while (curChildNode!=NULL)
				{
					if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Cell"))) 
					{
						xmlChar* szAttr;
						propNodePtr = curChildNode;
					xmlAttrPtr attrPtr = propNodePtr->properties;
					if (!xmlStrcmp(attrPtr->name, BAD_CAST "cellv"))
					{			 
						 szAttr = xmlGetProp(propNodePtr,BAD_CAST "cellv");
						
					}


						double cell[8];
						char a=*((char*)szAttr);
						szKey = xmlNodeGetContent(curChildNode);
						sscanf((char*)szKey,"%lf %lf %lf %lf %lf %lf %lf %lf",&cell[0],&cell[1],&cell[2],&cell[3],&cell[4],&cell[5],&cell[6],&cell[7]); 
						data->AddCell(cell,a);
						xmlFree(szAttr);
					}

					curChildNode=curChildNode->next;
				}
			}
		}
	}
	return true;
}


bool GeometryXMLReader::ReadRectifiedGrid(GMStructuredPoints* data,xmlNode* geometryNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量

	//取出节点中的内容
	if(data)
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"RectifiedGrid"))) 
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"limits"))) 
				{
					xmlNodePtr originNode = curChildNode->xmlChildrenNode;
					while(originNode != NULL) 
					{
						if ((!xmlStrcmp(originNode->name, (const xmlChar *)"GridEnvelope"))) 
						{
							xmlNodePtr pointNode = originNode->xmlChildrenNode;
							double lx=0,ly=0,lz=0;
							double hx=1,hy=1,hz=1;
							while(pointNode != NULL) 
							{
								if ((!xmlStrcmp(pointNode->name, (const xmlChar *)"low")))
								{
									szKey = xmlNodeGetContent(pointNode);

									sscanf((char*)szKey,"%lf %lf %lf",&lx,&ly,&lz); 	

									xmlFree(szKey);
								}

								if ((!xmlStrcmp(pointNode->name, (const xmlChar *)"high")))
								{
									szKey = xmlNodeGetContent(pointNode);

									sscanf((char*)szKey,"%lf %lf %lf",&hx,&hy,&hz); 	

									xmlFree(szKey);
								}
								pointNode = pointNode->next; 

							}


							data->SetDimensions(hx-lx+1,hy-ly+1,hz-lz+1);
						}

						originNode = originNode->next; 
					}

				}


				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"axisLabels"))) 
				{

				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"origin"))) 
				{
					xmlNodePtr originNode = curChildNode->xmlChildrenNode;
					while(originNode != NULL) 
					{
						if ((!xmlStrcmp(originNode->name, (const xmlChar *)"point"))) 
						{
							xmlNodePtr pointNode = originNode->xmlChildrenNode;
							while(pointNode != NULL) 
							{
								if ((!xmlStrcmp(pointNode->name, (const xmlChar *)"pos")))
								{
									szKey = xmlNodeGetContent(pointNode);
									double x,y,z;
									sscanf((char*)szKey,"%lf %lf %lf",&x,&y,&z); 	
									data->SetOrigin( x,y,z);
									xmlFree(szKey);
								}
								pointNode = pointNode->next; 

							}
						}

						originNode = originNode->next; 
					}
				}


				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"offsetVector"))) 
				{
					szKey = xmlNodeGetContent(curChildNode);
					double x,y,z;
					sscanf((char*)szKey,"%lf %lf %lf",&x,&y,&z);


					data->SetSpacing( x,y,z);
					xmlFree(szKey);
				}

				curChildNode = curChildNode->next; 
			}		  
		} 
		return 1;
}

vtkObject* GeometryXMLReader::ReadGeoVolume(xmlNode* geometryNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量

	GMUnstructuredGrid *tin = 0;
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"GeoVolume"))) 
	{
		tin = GMUnstructuredGrid::New();
		//
		//查找带有属性attribute的节点
		if (xmlHasProp(curNode,BAD_CAST "id"))
		{
			propNodePtr = curNode;
			xmlAttrPtr attrPtr = propNodePtr->properties;
			if (!xmlStrcmp(attrPtr->name, BAD_CAST "id"))
			{			 
				xmlChar* attr_str = xmlGetProp(propNodePtr, BAD_CAST "id");
				char* out = NULL;
				if(std::string(mXMLCoding) == "UTF-8")
					out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
				std::string sr = "";
				if (out)
					sr=std::string((char*)out);
				else
					sr=std::string((char*)attr_str);

				tin->SetID(sr);
				xmlFree(attr_str);
			}

		}



		if (xmlHasProp(curNode,BAD_CAST "dt"))
		{


			BinaryExport=1;
		}

		else
		{
			BinaryExport=0;
		}

		if(BinaryExport)
		{
			
			szKey=xmlNodeGetContent(curNode);
			char* buffer=(char*)szKey;

			

		     char cellTypecode[sizeof(int)*16];
			 Base64Decode(cellTypecode,buffer+37,8);	
			 int celltype=*((int *)(&cellTypecode));
			
			 if(celltype==4)
			 {
				 tin->SetGeometryType(10);
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


				 tin->SetPoints(points);
				 for(int i=0;i<cellcount;i++)
				 {

					 char cell[sizeof(vtkIdType)*4*4];
					 Base64Decode(cell,buffer+45+(pointcount*32)+(i*24),24);
					 vtkIdType a=*((vtkIdType*)(&cell));
					 vtkIdType b=*(((vtkIdType*)(&cell))+1);
					 vtkIdType c=*(((vtkIdType*)(&cell))+2);
					 vtkIdType d=*(((vtkIdType*)(&cell))+3);
					 vtkIdType ncell[4] ={a,b,c,d};
					 tin->InsertNextCell(10,4,ncell);
				 }

				// tin->SetPoints(points);
				// tin->SetPolys(cellarray);
			 }

			 if(celltype==8)
			 {
				 tin->SetGeometryType(12);
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


				 tin->SetPoints(points);
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
					 tin->InsertNextCell(12,8,ncell);
				 }
			 }

			
		}

		else
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL && tin) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Vertices"))) 
				{
					ReadVertices(tin,curChildNode);
				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Polyhedrons"))) 
				{
					//tin->SetGeometryType(1);
					ReadCells(tin,curChildNode);
				}

				curChildNode = curChildNode->next; 
			}		  
		}
		
	}   


	return tin;
}

vtkObject* GeometryXMLReader::ReadPolyDataTIN(xmlNode* geometryNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量

	GMPolyData *tin = 0;
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"GeoTin"))) 
	{
		tin = GMPolyData::New();
		tin->SetGeometryType(5);
		//查找带有属性attribute的节点
		if (xmlHasProp(curNode,BAD_CAST "id"))
		{
			propNodePtr = curNode;
			xmlAttrPtr attrPtr = propNodePtr->properties;

			if (!xmlStrcmp(attrPtr->name, BAD_CAST "id"))
			{		

				xmlChar* attr_str = xmlGetProp(propNodePtr, BAD_CAST "id");
				char* out = NULL;
				if(std::string(mXMLCoding) == "UTF-8")
					out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
				std::string sr = "";
				if (out)
					sr=std::string((char*)out);
				else
					sr=std::string((char*)attr_str);
		
				tin->SetID(sr);
				xmlFree(attr_str);
			}

			//BinaryExport=0;
		}

		if (xmlHasProp(curNode,BAD_CAST "dt"))
		{
			

			BinaryExport=1;
		}

		else
		{
			BinaryExport=0;
		}
		
		//查找带有属性binaryid的节点
		/*if (xmlHasProp(curNode,BAD_CAST"binaryid"))
		{
			propNodePtr = curNode;
			xmlAttrPtr attrPtr = propNodePtr->properties;
			if (!xmlStrcmp(attrPtr->name, BAD_CAST "binaryid"))
			{			 
				xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "binaryid");
				tin->SetID((char*)szAttr);
				xmlFree(szAttr);
			}

			BinaryExport=1;
		}*/


		if(BinaryExport)
		{
			szKey=xmlNodeGetContent(curNode);
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
			/*char byteOrder = 0;
			char byteOrdercode[sizeof(int)];
			Base64Decode(byteOrdercode, buf, sizeof(char)*16);
			int a=*((int *)(&byteOrdercode));
			cout<<a<<endl;*/

			
			


		}

		else
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL && tin) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Vertices"))) 
				{
					ReadVertices(tin,curChildNode);
				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Triangles"))) 
				{
					vtkCellArray *line = vtkCellArray::New();
					tin->SetPolys(line);
					ReadTriangles(tin,curChildNode);
				}

				curChildNode = curChildNode->next; 
			}
		}
		
			  
	}   


	/*
	if(tin)
	{	
		vtkPolyDataNormals *gn = vtkPolyDataNormals::New();
		gn->SetInput(tin);
		gn->NonManifoldTraversalOff();

		gn->Update();

		tin->GetPointData()->SetNormals( gn->GetOutput()->GetPointData()->GetNormals());

		gn->Delete();
	}*/
	return tin;
}

bool GeometryXMLReader::ReadVertices(vtkPointSet* polydata,xmlNode* VerticesNode)
{
	xmlChar *szKey;          //临时字符串变量
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = VerticesNode->xmlChildrenNode;
	xmlNodePtr propNodePtr = curNode;

	while(curNode != NULL) 
	{
		//取出节点中的内容
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Vertex"))) 
		{ 
			//查找带有属性attribute的节点
			int vid = -1;
			if (xmlHasProp(curNode,BAD_CAST "IndexNo"))
			{
				propNodePtr = curNode;
				xmlAttrPtr attrPtr = propNodePtr->properties;

				while(attrPtr)
				{
					if (!xmlStrcmp(attrPtr->name, BAD_CAST "IndexNo"))
					{
						xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "IndexNo");
						//ReadGeoMap((unsigned char*)szAttr);
						if(szAttr)
							vid = atoi((char*)szAttr);
						xmlFree(szAttr);
					}

					attrPtr = attrPtr->next;
				}
			}

			//对于JSON格式的杭州模型，14_0t_mod.xml的16620 - 16625、1_0t_mod.xml的14198和2_0t_mod.xml的12464 - 12472的坐标值有误
			if((vid >= 16620 && vid <= 16625) || vid == 14198 || (vid >= 12464 && vid <= 12472))
			{
				szKey = xmlNodeGetContent(curNode);

				double x = 0, y = 0, z = 0;
				char buf[256] = {'\0'};
				int cc = 0;
				for(int ii = 0; ii < strlen((char*)szKey); ii++)
				{
					if(szKey[ii] != ',')
						buf[cc++] = szKey[ii];
				}
				sscanf(buf,"%lf %lf %lf",&x,&y,&z);

				vtkPoints* points  = polydata->GetPoints();

				if(!points)
				{ 
					points = vtkPoints::New();
					points->SetDataTypeToDouble();
					polydata->SetPoints(points);
				}

				points->InsertPoint(vid,x,y,z);
			}

			else
			{

				szKey = xmlNodeGetContent(curNode);

				double x = 0, y = 0, z = 0;
				char buf[256] = {'\0'};
				int cc = 0;
				for(int ii = 0; ii < strlen((char*)szKey); ii++)
				{
					if(szKey[ii] != ',')
						buf[cc++] = szKey[ii];
				}
				sscanf(buf,"%lf %lf %lf",&x,&y,&z);

				vtkPoints* points  = polydata->GetPoints();

				if(!points)
				{
					points = vtkPoints::New();
					points->SetDataTypeToDouble();
					polydata->SetPoints(points);
				}

				points->InsertPoint(vid,x,y,z);
				// polydata->AddPoint(pnt);

			}
			xmlFree(szKey);		 
		} 

		curNode = curNode->next; 
	}
	return 1;
}

bool GeometryXMLReader::ReadCells(GMUnstructuredGrid* polydata,xmlNode* TrisNode)
{
	bool a= 0;
	xmlChar *szKey;          //临时字符串变量
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = TrisNode->xmlChildrenNode;
	xmlNodePtr propNodePtr = curNode; 
	while(curNode != NULL) 
	{
		//取出节点中的内容
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Tetrahedron"))) 
		{
			polydata->SetGeometryType(10);
			int TriID = -9999;
			int x = -1,y = -1,z = -1,t=-1;
			int t1=-1,t2=-1,t3 = -1,t4=-1;
			//查找带有属性attribute的节点
			if (xmlHasProp(curNode,BAD_CAST "IndexNo"))
			{
				propNodePtr = curNode;
				xmlAttrPtr attrPtr = propNodePtr->properties;
				if (!xmlStrcmp(attrPtr->name, BAD_CAST "IndexNo"))
				{
					xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "IndexNo");
					//ReadGeoMap((unsigned char*)szAttr);
					TriID = atoi((char*)szAttr);
					xmlFree(szAttr);
				}
			}

			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"VertexList"))) 
				{
					szKey = xmlNodeGetContent(curChildNode);

					sscanf((char*)szKey,"%d %d %d %d",&x,&y,&z,&t);  
					xmlFree(szKey);
				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"NeighborList"))) 
				{
					szKey = xmlNodeGetContent(curChildNode);
					sscanf((char*)szKey,"%d %d %d %d",&t1,&t2,&t3,&t4);  
					xmlFree(szKey);
				}

				curChildNode = curChildNode->next; 
			}

			vtkIdType cell[4] ={x,y,z,t};
			polydata->InsertNextCell(10,4,cell); 

		} 


		//取出节点中的内容
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Cuboid"))) 
		{
			polydata->SetGeometryType(12);
			int TriID = -9999;
			vtkIdType cell[8]  ;
			//int x = -1,y = -1,z = -1,t=-1;
			//int t1=-1,t2=-1,t3 = -1,t4=-1;
			//查找带有属性attribute的节点
			if (xmlHasProp(curNode,BAD_CAST "IndexNo"))
			{
				propNodePtr = curNode;
				xmlAttrPtr attrPtr = propNodePtr->properties;
				if (!xmlStrcmp(attrPtr->name, BAD_CAST "IndexNo"))
				{
					xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "IndexNo");
					//ReadGeoMap((unsigned char*)szAttr);
					TriID = atoi((char*)szAttr);
					xmlFree(szAttr);
				}
			}

			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"VertexList"))) 
				{
					szKey = xmlNodeGetContent(curChildNode);

					sscanf((char*)szKey,"%d %d %d %d %d %d %d %d",&cell[0],&cell[1],&cell[2],&cell[3],&cell[4],&cell[5],&cell[6],&cell[7]);  
					xmlFree(szKey);


					polydata->InsertNextCell(12,8,cell);  
				}

				curChildNode = curChildNode->next; 
			}
		} 

		curNode = curNode->next;  
	}

	return 1;
}

bool GeometryXMLReader::ReadTriangles(GMPolyData* polydata,xmlNode* TrisNode)
{
	xmlChar *szKey;          //临时字符串变量
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = TrisNode->xmlChildrenNode;
	xmlNodePtr propNodePtr = curNode; 
	while(curNode != NULL) 
	{
		//取出节点中的内容
		if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Triangle"))) 
		{
			int TriID = -9999;
			int x = -1,y = -1,z = -1;
			int t1=-1,t2=-1,t3 = -1;
			//查找带有属性attribute的节点
			if (xmlHasProp(curNode,BAD_CAST "IndexNo"))
			{
				propNodePtr = curNode;
				xmlAttrPtr attrPtr = propNodePtr->properties;
				if (!xmlStrcmp(attrPtr->name, BAD_CAST "IndexNo"))
				{
					xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "IndexNo");
					//ReadGeoMap((unsigned char*)szAttr);
					TriID = atoi((char*)szAttr);
					xmlFree(szAttr);
				}
			}

			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"VertexList"))) 
				{
					szKey = xmlNodeGetContent(curChildNode);



					sscanf((char*)szKey,"%d %d %d",&x,&y,&z);  

					xmlFree(szKey);
				}

				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"NeighborList"))) 
				{
					szKey = xmlNodeGetContent(curChildNode);
					sscanf((char*)szKey,"%d %d %d",&t1,&t2,&t3);  
					xmlFree(szKey);
				}

				curChildNode = curChildNode->next; 
			}

			vtkCellArray* line = polydata->GetPolys ();



			line->InsertNextCell(3);
			line->InsertCellPoint(x);
			line->InsertCellPoint(y);
			line->InsertCellPoint(z);

		} 

		curNode = curNode->next;  
	}
	return 1;
}

vtkObject* GeometryXMLReader::ReadPolyDataTEN(xmlNode* geometryNode)
{
	return 0;
}

///@brief  读取<LineString>标签
///@param  geometryNode <LineString>标签的xml结点指针
///@return 将LineString保存为vtkObject返回
///@author zhaowei
///@date   2013.11.19
vtkObject* GeometryXMLReader::ReadPolyDataLine(xmlNode* geometryNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量
	GMPolyData* Line = NULL;

	//取多点数据
	if((!xmlStrcmp(curNode->name, (const xmlChar *)"MultiCurve")))
	{
		vtkPoints* points =vtkPoints::New();
		vtkCellArray* liness  = vtkCellArray::New();

		int numcountpoint=0;
		if (xmlHasProp(curNode,BAD_CAST "id"))
		{			  
			Line = GMPolyData::New();

			Line->SetGeometryType(4);

			xmlChar* attr_str = xmlGetProp(curNode, BAD_CAST "id");
			char* out = NULL;
			if(std::string(mXMLCoding) == "UTF-8")
				out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
			std::string sr = "";
			if (out)
				sr=std::string((char*)out);
			else
				sr=std::string((char*)attr_str);

			Line->SetID(sr);
			xmlFree(attr_str);
		}

		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode != NULL && Line) 
		{				
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"curveMember"))) 
			{
				xmlNodePtr curChildNode2 = curChildNode->xmlChildrenNode;
				while(curChildNode2!=NULL &&Line)
				{

					if ((!xmlStrcmp(curChildNode2->name, (const xmlChar *)"LineString"))) 
					{
						xmlNodePtr curChildNode3= curChildNode2->xmlChildrenNode;
						while(curChildNode3 != NULL && Line) 
						{				
							if ((!xmlStrcmp(curChildNode3->name, (const xmlChar *)"posList"))) 
							{
								int PointCount  = 0;
								//查找带有属性attribute的节点
								if (xmlHasProp(curChildNode3,BAD_CAST "count"))
								{						   
									xmlChar* szAttr = xmlGetProp(curChildNode3,BAD_CAST "count");
									PointCount = atoi((char*)szAttr);
									xmlFree(szAttr);
								}

								if(PointCount>0)
								{

									szKey = xmlNodeGetContent(curChildNode3);      
									std::string pointlst((char*)szKey);


									std::istringstream istr((char*)szKey);
									for(int i =0 ; i < PointCount; i++)
									{
										double x,y,z ;
										istr>>x; istr>>y; istr>>z; 
										//datapointer[i] =a;
										points->InsertNextPoint(x,y,z);	
									}

									xmlFree(szKey); 
							
									liness->InsertNextCell(PointCount);
									for(int i = 0; i < PointCount; i++)
									{
										
										liness->InsertCellPoint(i+numcountpoint);
				
									}
									numcountpoint=numcountpoint+PointCount;
								}
								else
								{
									std::string error = "非法数据：线的点个数为0" + Line->GetID();
									gmmlDisplayErrorandWarningText(error.c_str());
								}
							}
							curChildNode3 = curChildNode3->next; 
						}
					}

					curChildNode2 = curChildNode2->next; 
				}
			}

			curChildNode = curChildNode->next; 
		}

		Line->SetPoints(points);
		Line->SetLines(liness);
	}

	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"LineString"))) 
	{

		//查找带有属性attribute的节点
		if (xmlHasProp(curNode,BAD_CAST "id"))
		{			  
			Line = GMPolyData::New();
			Line->SetGeometryType(3);
			vtkPoints* points  = Line->GetPoints();


			xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "id");
			Line->SetID((char*)szAttr);
			xmlFree(szAttr);

		}


		if (xmlHasProp(curNode,BAD_CAST "dt"))
		{


			BinaryExport=1;
		}

		else
		{
			BinaryExport=0;
		}


		if(BinaryExport)
		{
			szKey=xmlNodeGetContent(curNode);
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

			Line->SetPoints(points);
			Line->SetLines(cellarray);

		}

		else
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL && Line) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"posList"))) 
				{
					int PointCount  = 0;
					//查找带有属性attribute的节点
					if (xmlHasProp(curChildNode,BAD_CAST "count"))
					{						   
						xmlChar* szAttr = xmlGetProp(curChildNode,BAD_CAST "count");
						PointCount = atoi((char*)szAttr);
						xmlFree(szAttr);
					}

					if(PointCount>0)
					{

						szKey = xmlNodeGetContent(curChildNode);      
						std::string pointlst((char*)szKey);


						std::istringstream istr((char*)szKey);
						for(int i =0 ; i < PointCount; i++)
						{
							double x,y,z ;
							istr>>x; istr>>y; istr>>z; 
							//datapointer[i] =a;

							vtkPoints* points  = Line->GetPoints();

							if(!points)
							{ 
								points = vtkPoints::New();
								points->SetDataTypeToDouble();
								Line->SetPoints(points);

								vtkCellArray*  lines = vtkCellArray::New();
								Line->SetLines(lines);
							}
							points->InsertPoint(i,x,y,z);	
						}

						xmlFree(szKey); 
						vtkCellArray* lines  = Line->GetLines();
						for(int i = 0; i < PointCount-1; i++)
						{
							lines->InsertNextCell(2);
							lines->InsertCellPoint(i);
							lines->InsertCellPoint(i+1);
						}
					}
					else
					{
						std::string error = "非法数据：线的点个数为0" + Line->GetID();
						gmmlDisplayErrorandWarningText(error.c_str());
					}
				}
				curChildNode = curChildNode->next; 
			}
		}

		

		if(Line)
	   {
		   vtkPoints* points  = Line->GetPoints();
		   if(!points || points->GetNumberOfPoints() < 1)
		   {
			std::string error = "非法数据：线的点个数为0" + Line->GetID();
			gmmlDisplayErrorandWarningText(error.c_str());
		   }
		}
	}	 
	return Line;
}

vtkObject* GeometryXMLReader::ReadPolyDataPoints(xmlNode* geometryNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量

	GMPolyData* point =0;

	//取多点数据
	if((!xmlStrcmp(curNode->name, (const xmlChar *)"MultiPoint")))
	{
		vtkPoints* points =vtkPoints::New();
		points->SetDataTypeToDouble();
		vtkCellArray* lines  = vtkCellArray::New();

		int numcountpoint=0;
		if (xmlHasProp(curNode,BAD_CAST "id"))
		{			  
			point = GMPolyData::New();

			point->SetGeometryType(2);

			xmlChar* attr_str = xmlGetProp(curNode, BAD_CAST "id");
			char* out = NULL;
			if(std::string(mXMLCoding) == "UTF-8")
				out =  ConvertEnc("UTF-8","GB2312", (char*)attr_str);
			std::string sr = "";
			if (out)
				sr=std::string((char*)out);
			else
				sr=std::string((char*)attr_str);

			point->SetID(sr);
			xmlFree(attr_str);
		}

		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode != NULL && point) 
		{				
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"pointMember"))) 
			{
				xmlNodePtr curChildNode2 = curChildNode->xmlChildrenNode;
				while(curChildNode2!=NULL &&point)
				{

					if ((!xmlStrcmp(curChildNode2->name, (const xmlChar *)"Point"))) 
					{
						xmlNodePtr curChildNode3= curChildNode2->xmlChildrenNode;
						while(curChildNode3 != NULL && point) 
						{				
							if ((!xmlStrcmp(curChildNode3->name, (const xmlChar *)"pos"))) 
							{
								int PointCount  = 0;
								//查找带有属性attribute的节点
								if (xmlHasProp(curChildNode3,BAD_CAST "srsDimension"))
								{						   
									xmlChar* szAttr = xmlGetProp(curChildNode3,BAD_CAST "srsDimension");
									PointCount = atoi((char*)szAttr);
									xmlFree(szAttr);
								}

								szKey = xmlNodeGetContent(curChildNode3);      

								if(PointCount>=2)
								{


									double x,y,z=0;
									//char buf[64];
									//pointlst.find(" "
									//	double x,y,z;
									char buf[256];
									int cc = 0;
									for(int ii = 0; ii < strlen((char*)szKey); ii++)
									{
										if(szKey[ii] != ',')
											buf[cc++] = szKey[ii];
									}
									sscanf(buf,"%lf %lf %lf",&x,&y,&z);

									//sscanf(pointlst.c_str(),"%lf %lf %lf",&x,&y,&z);


									
									points->InsertNextPoint(x,y,z);	
									numcountpoint++;

								}

								xmlFree(szKey);
							}

							curChildNode3 = curChildNode3->next; 
						}
					}

					curChildNode2 = curChildNode2->next; 
				}
			}

			curChildNode = curChildNode->next; 
		}

		

		for(int i = 0; i < numcountpoint; i++)
		{
			lines->InsertNextCell(1);
			lines->InsertCellPoint(i);					 
		}

		point->SetPoints(points);
		point->SetVerts(lines);
	}


	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Point"))) 
	{

		if (xmlHasProp(curNode,BAD_CAST "id"))
		{			  
			point = GMPolyData::New();

			point->SetGeometryType(1);

			xmlChar* szAttr = xmlGetProp(curNode,BAD_CAST "id");

			point->SetID((char*)szAttr);
			xmlFree(szAttr);

		}

		if (xmlHasProp(curNode,BAD_CAST "dt"))
		{


			BinaryExport=1;
		}

		else
		{
			BinaryExport=0;
		}

		if(BinaryExport)
		{
			szKey=xmlNodeGetContent(curNode);
			char* buffer=(char*)szKey;


			char pointnum[sizeof(int)*4];
			Base64Decode(pointnum, buffer+13, 8);
			int pointcount=*((int *)(&pointnum));


			vtkPoints*points=vtkPoints::New();
			vtkCellArray*cellarray=vtkCellArray::New();
			for(int i=0;i<1;i++)
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
			
		}

		else
		{
			xmlNodePtr curChildNode = curNode->xmlChildrenNode;
			while(curChildNode != NULL && point) 
			{				
				if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"pos"))) 
				{
					int PointCount  = 0;
					//查找带有属性attribute的节点
					if (xmlHasProp(curChildNode,BAD_CAST "srsDimension"))
					{						   
						xmlChar* szAttr = xmlGetProp(curChildNode,BAD_CAST "srsDimension");
						PointCount = atoi((char*)szAttr);
						xmlFree(szAttr);
					}

					szKey = xmlNodeGetContent(curChildNode);      

					if(PointCount>=2)
					{


						double x,y,z=0;
						//char buf[64];
						//pointlst.find(" "
						//	double x,y,z;
						char buf[256];
						int cc = 0;
						for(int ii = 0; ii < strlen((char*)szKey); ii++)
						{
							if(szKey[ii] != ',')
								buf[cc++] = szKey[ii];
						}
						sscanf(buf,"%lf %lf %lf",&x,&y,&z);

						//sscanf(pointlst.c_str(),"%lf %lf %lf",&x,&y,&z);


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

					xmlFree(szKey);

					vtkCellArray* lines  = point->GetVerts();

					for(int i = 0; i < 1; i++)
					{
						lines->InsertNextCell(1);
						lines->InsertCellPoint(0);					 
					}
				}

				curChildNode = curChildNode->next; 
			}
		}

	

	}		 	       
	return point;
}

void GeometryXMLReader::ReadProperty(xmlNode* arrayNode)
{
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = arrayNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量

	vtkDataArray* data = 0;
	std::string datatype ;
	int datacount = 0;
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"ShapeProperty"))) 
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode)
		{
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"GeoDiscreteCoverage"))) 
			{
				std::vector<vtkDataArray*> dataarrays;
				std::vector<int> dataarraytypes;
				std::vector<std::string> dataarraysname;
				ReadDiscreteCoverage(curChildNode, dataarrays, dataarraytypes, dataarraysname);
			}
			curChildNode = curChildNode->next;
		}
	}
	
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"PropertyModels"))) 
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode)
		{
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"GeoDiscreteCoverage"))) 
			{
				std::vector<vtkDataArray*> dataarrays;
				std::vector<int> dataarraytypes;
				std::vector<std::string> dataarraysname;
				ReadDiscreteCoverageGMML(curChildNode);
			}
			curChildNode = curChildNode->next;
		}
	}
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"Propertymodels"))) 
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode)
		{
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"GeoDiscreteCoverage"))) 
			{
				std::vector<vtkDataArray*> dataarrays;
				std::vector<int> dataarraytypes;
				std::vector<std::string> dataarraysname;
				ReadDiscreteCoverageGMML(curChildNode);
			}
			curChildNode = curChildNode->next;
		}
	}
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"GeometryProperty"))) 
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode)
		{
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"GeoDiscreteCoverage"))) 
			{
				std::vector<vtkDataArray*> dataarrays;
				std::vector<int> dataarraytypes;
				std::vector<std::string> dataarraysname;
				ReadDiscreteCoverage(curChildNode, dataarrays, dataarraytypes, dataarraysname);
			}
			curChildNode = curChildNode->next;
		}
	}

}

vtkDataArray*  GeometryXMLReader::ReadValueArray(xmlNode* arrayNode)
{	
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = arrayNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量
	GMPolyData* point = 0;

	vtkDataArray* data = 0;
	std::string datatype ;
	int datacount = 0;
	int datacompont;
	//取出节点中的内容

	std::vector<double> dataarray;
	int dataType=0; // 0:double   1:long  

	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"ValueArray"))) 
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode != NULL) 
		{

			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"valueComponents"))) 
			{				
				xmlNodePtr curQuantityNode = curChildNode->xmlChildrenNode;
				while(curQuantityNode != NULL) 
				{
					if ((!xmlStrcmp(curQuantityNode->name, (const xmlChar *)"Quantity"))) 
					{
						dataType = 0;
						szKey = xmlNodeGetContent(curQuantityNode);						
                        dataarray.push_back(atof((const char *)szKey));
						xmlFree(szKey);
					}

					if ((!xmlStrcmp(curQuantityNode->name, (const xmlChar *)"Count"))) 
					{
						dataType =1;
						szKey = xmlNodeGetContent(curQuantityNode);						
						dataarray.push_back(atoi((const char *)szKey));
						xmlFree(szKey);
					}

					if ((!xmlStrcmp(curQuantityNode->name, (const xmlChar *)"ValueArray"))) 
					{
						xmlNodePtr curArrayNode = curQuantityNode->xmlChildrenNode;
						while(curArrayNode != NULL)
						{
							if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"valueComponents"))) 
							{
								xmlNodePtr curCountNode = curChildNode->xmlChildrenNode;
								while(curCountNode != NULL) 
								{
									if ((!xmlStrcmp(curCountNode->name, (const xmlChar *)"Quantity"))) 
									{
										dataType = 0;
										szKey = xmlNodeGetContent(curQuantityNode);						
										dataarray.push_back(atof((const char *)szKey));
										xmlFree(szKey);
									}

									if ((!xmlStrcmp(curCountNode->name, (const xmlChar *)"Count"))) 
									{
										dataType =1;
										szKey = xmlNodeGetContent(curQuantityNode);						
										dataarray.push_back(atoi((const char *)szKey));
										xmlFree(szKey);
									}
									curCountNode = curCountNode->next;
								}
							}
							curArrayNode = curArrayNode->next;
						}
					}
					curQuantityNode = curQuantityNode->next;
				}

			}
			curChildNode = curChildNode->next;
		}

	}

	if(dataType == 1)
	{
		vtkLongArray * vtkdataarray = vtkLongArray::New();
		data = vtkdataarray;
		
		long *datavalues = new long[dataarray.size()];
		
		for(int i = 0; i <dataarray.size(); i++)
			datavalues[i] = dataarray[i];
		vtkdataarray->SetArray(datavalues,dataarray.size(), 0);
	}

	if(dataType == 0)
	{
		vtkDoubleArray * vtkdataarray = vtkDoubleArray::New();
		data = vtkdataarray;

		double *datavalues = new double[dataarray.size()];

		for(int i = 0; i <dataarray.size(); i++)
			datavalues[i] = dataarray[i];
		vtkdataarray->SetArray(datavalues,dataarray.size(),0);
	}

	return data;
}

vtkDataArray*  GeometryXMLReader::ReadDataArray(xmlNode* arrayNode)
{
	//vtkDataArray *dataarray = (vtkDataArray *)Feature_;
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = arrayNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量
	GMPolyData* point =0;

	vtkDataArray* data = 0;
	std::string datatype ;
	int datacount = 0;
	int datacompont;
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"DataArray"))) 
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode != NULL) 
		{
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"DataType"))) 
			{
				szKey = xmlNodeGetContent(curChildNode);
				int leftc = TrimLeft((char*)szKey);
				int rightc = TrimRight((char*)szKey);
				int len = strlen((char*)szKey);
				datatype = (char*)szKey;
				datatype = datatype.substr(leftc,len -rightc-leftc);
				xmlFree(szKey); 
			}
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"DataSize"))) 
			{
				szKey = xmlNodeGetContent(curChildNode);
				datacount = atoi((char*)szKey);
				xmlFree(szKey); 
			}
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"Data"))) 
			{
				szKey = xmlNodeGetContent(curChildNode);
				if(!data && datacount>0)
				{
					if(datatype == "double")
					{
						vtkDoubleArray *doubledata = vtkDoubleArray::New();
						double* datapointer =  doubledata->WritePointer(0,datacount);

						data = doubledata;
						char* buf = (char*)szKey;
						int offset = 0;

						std::istringstream istr(buf);; 
						for(int i =0 ; i < datacount; i++)
						{
							double a ;
							istr>>a; 
							datapointer[i] =a;
						}

					}

					if(datatype == "float")
					{
						vtkFloatArray *doubledata = vtkFloatArray::New();
						float* datapointer =  doubledata->WritePointer(0,datacount);
						data = doubledata;
						std::istringstream istr((char*)szKey);; 
						for(int i =0 ; i < datacount; i++)
						{
							float a ;
							istr>>a; 
							datapointer[i] =a;
						}
					}

					if(datatype == "char")
					{
						vtkCharArray *doubledata = vtkCharArray::New();
						char*  datapointer =  doubledata->WritePointer(0,datacount);
						data = doubledata;
						char* buf = (char*)szKey;

						std::istringstream istr((char*)szKey);; 
						for(int i =0 ; i < datacount; i++)
						{
							short a ;
							istr>>a; 
							datapointer[i] =a;
						}
					}
					if(datatype == "unsignedchar")
					{
						vtkUnsignedCharArray *doubledata = vtkUnsignedCharArray::New();
						unsigned char*  datapointer = doubledata->WritePointer(0,datacount);
						data = doubledata;
						char* buf = (char*)szKey;

						std::istringstream istr((char*)szKey);; 
						for(int i =0 ; i < datacount; i++)
						{
							short a ;
							istr>>a; 
							datapointer[i] =a;
						}
					}
					if(datatype == "unsigned char")
					{
						vtkUnsignedCharArray *doubledata = vtkUnsignedCharArray::New();
						unsigned char*  datapointer = doubledata->WritePointer(0,datacount);
						data = doubledata;
						char* buf = (char*)szKey;

						std::istringstream istr((char*)szKey);; 
						for(int i =0 ; i < datacount; i++)
						{
							short a ;
							istr>>a; 
							datapointer[i] =a;
						}
					}
					if(datatype == "long" || datatype == "short" || datatype == "int")
					{
						vtkLongArray *doubledata = vtkLongArray::New();
						long*  datapointer =doubledata->WritePointer(0,datacount);
						data = doubledata;
						char* buf = (char*)szKey;
						std::istringstream istr((char*)szKey);; 
						for(int i =0 ; i < datacount; i++)
						{
							long a ;
							istr>>a; 
							datapointer[i] =a;
						}
					}


				}

				xmlFree(szKey); 
			}

			curChildNode = curChildNode->next;
		}

	}

	return data;
}
vtkDataArray*  GeometryXMLReader::ReadDiscreteCoverageGMML(xmlNode* geometryNode)
{
	//vtkDataArray *dataarray = (vtkDataArray *)Feature_;
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量
	GMPolyData* point =0;
	vtkDataArray*data = 0;

	std::string PropertyPosType;
	std::string PropertyFeildType;
	std::string PropertyFeildName;
	int compount = 1;

	
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"GeoDiscreteCoverage"))) 
	{
		xmlNodePtr curChildNode = curNode->xmlChildrenNode;
		while(curChildNode != NULL) 
		{
			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"domainSet"))) 
			{

			}

			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"rangeType"))) 
			{
				xmlNodePtr curRangeNode = curChildNode->xmlChildrenNode;
				while(curRangeNode != NULL) 
				{
					if ((!xmlStrcmp(curRangeNode->name, (const xmlChar *)"DataRecord"))) 
					{
						xmlNodePtr curFieldNode = curRangeNode->xmlChildrenNode;
						while(curFieldNode != NULL) 
						{
							if ((!xmlStrcmp(curFieldNode->name, (const xmlChar *)"field"))) 
							{
								propNodePtr = curFieldNode;
								xmlAttrPtr attrPtr = curFieldNode->properties;
								xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "name");
								PropertyFeildName = (char*)szAttr;
								xmlFree(szAttr);  
							}
							curFieldNode = curFieldNode->next;
						}
					}
					curRangeNode = curRangeNode->next;
				}
			}

			if ((!xmlStrcmp(curChildNode->name, (const xmlChar *)"GeoPropertyRange"))) 
			{
				xmlNodePtr curRangeNode = curChildNode->xmlChildrenNode;
				while(curRangeNode != NULL) 
				{
					if ((!xmlStrcmp(curRangeNode->name, (const xmlChar *)"PropertyPosType"))) 
					{
						szKey = xmlNodeGetContent(curRangeNode);
						int leftc = TrimLeft((char*)szKey);
						int rightc = TrimRight((char*)szKey);
						int len = strlen((char*)szKey);
						PropertyPosType = (char*)szKey;
						PropertyPosType = PropertyPosType.substr(leftc,len -rightc-leftc);
						xmlFree(szKey); 
					}

					if ((!xmlStrcmp(curRangeNode->name, (const xmlChar *)"GeoPropertyDataType"))) 
					{
						xmlNodePtr curFeildNode = curRangeNode->xmlChildrenNode;
						while(curFeildNode != NULL) 
						{
							if ((!xmlStrcmp(curFeildNode->name, (const xmlChar *)"PropertyFeildType"))) 
							{
								propNodePtr = curFeildNode;
								xmlAttrPtr attrPtr = curFeildNode->properties;
								xmlChar* szAttr = xmlGetProp(propNodePtr,BAD_CAST "Row");
								int Row = atoi((char*)szAttr);
								xmlFree(szAttr); 
								szAttr = xmlGetProp(propNodePtr,BAD_CAST "Col");
								int Col = atoi((char*)szAttr);

								compount = Row*Col;
								xmlFree(szAttr);

								szKey = xmlNodeGetContent(curFeildNode);
								int leftc = TrimLeft((char*)szKey);
								int rightc = TrimRight((char*)szKey);
								int len = strlen((char*)szKey);
								PropertyFeildType = (char*)szKey;
								PropertyFeildType = PropertyFeildType.substr(leftc,len -rightc-leftc);
								xmlFree(szKey);

							}

							curFeildNode = curFeildNode->next;
						}
					}

					if ((!xmlStrcmp(curRangeNode->name, (const xmlChar *)"Values"))) 
					{
						xmlNodePtr curFeildNode = curRangeNode->xmlChildrenNode;
						while(curFeildNode != NULL) 
						{
							vtkDataArray*curdata = ReadDataArray(curFeildNode);
							if(curdata)
								data = curdata;
							curFeildNode = curFeildNode->next;
						}

					}
					curRangeNode = curRangeNode->next;

				}
			}
			curChildNode = curChildNode->next;

		}
	}

	if(ShapeObject)
	if(data && data->GetSize() )
	{
		
		data->SetNumberOfComponents(compount);
		data->SetNumberOfTuples(data->GetSize()/compount);

		data->SetName(PropertyFeildName.c_str());
		vtkObject* mDataSet = ShapeObject;
		
		if(0 == strcmp(mDataSet->GetClassName(),"GMPolyData"))
		{
			GMPolyData* DataSet =(GMPolyData*)mDataSet;

			if( PropertyPosType == "VERTEX")
				DataSet->GetPointData()->AddArray(data);

			if( PropertyPosType == "FACE" || PropertyPosType == "VOLUME")
				DataSet->GetCellData()->AddArray(data);
		}
		if(0 == strcmp(mDataSet->GetClassName(), "GMStructuredPoints"))
		{
			GMStructuredPoints* DataSet =(GMStructuredPoints*)mDataSet;

			if( PropertyPosType == "VERTEX")
				DataSet->GetPointData()->AddArray(data);
			

			if( PropertyPosType == "FACE" || PropertyPosType == "VOLUME")
				DataSet->GetCellData()->AddArray(data);
		}

		if(0 == strcmp(mDataSet->GetClassName(), "GMUnstructuredGrid"))
		{
			GMUnstructuredGrid* DataSet =(GMUnstructuredGrid*)mDataSet;

			if( PropertyPosType == "VERTEX")
				DataSet->GetPointData()->AddArray(data);

			if( PropertyPosType == "FACE" || PropertyPosType == "VOLUME")
				DataSet->GetCellData()->AddArray(data);
		}

		if(0 == strcmp(mDataSet->GetClassName() , "GMCornerPointGrid"))
		{
			GMCornerPointGrid* DataSet =(GMCornerPointGrid*)mDataSet;

			/*if( PropertyPosType == "VERTEX")
				DataSet->GetPointData()->AddArray(data);

			if( PropertyPosType == "FACE" || PropertyPosType == "VOLUME")
				DataSet->GetCellData()->AddArray(data);*/

			DataSet->AddCellPropertyData(data);
		}
	}
	return data;


}

bool  GeometryXMLReader::ReadDiscreteCoverage(xmlNode* geometryNode,std::vector<vtkDataArray*> &dataarrays,std::vector<int> &dataarraytypes, std::vector<std::string>& dataarraysname)
{
	//vtkDataArray *dataarray = (vtkDataArray *)Feature_;
	xmlNodePtr curNode;      //定义结点指针(你需要它为了在各个结点间移动) 
	curNode = geometryNode;
	xmlNodePtr propNodePtr = curNode;
	xmlChar *szKey;          //临时字符串变量
	GMPolyData* point =0;
	vtkDataArray*data = 0;

	std::string PropertyPosType;
	std::string PropertyFieldType;
	std::string PropertyFieldName;
	int compount = 1;

	std::vector<vtkDataArray*> mydataarrays;
	std::vector<std::string> mydataarraysname;
	std::vector<int> mydataarraystype;
 
	//取出节点中的内容
	if ((!xmlStrcmp(curNode->name, (const xmlChar *)"GeoDiscreteCoverage"))) 
	{ 
		 xmlNodePtr curChildNode = curNode->children->next;
		 while(curChildNode != NULL)
		 {
			 if(!xmlStrcmp(curChildNode->name, (const xmlChar *)"rangeSet"))
			 {
				 xmlNodePtr curRangeNode = curChildNode->children;
				 while(curRangeNode != NULL)
				 {
					 if(!xmlStrcmp(curRangeNode->name, (const xmlChar *)"ValueArray"))
						 mydataarrays.push_back(ReadValueArray(curRangeNode));
					 curRangeNode = curRangeNode->next;
				 }				 
			 }
			 if(!xmlStrcmp(curChildNode->name, (const xmlChar *)"rangeType"))
			 {
				 xmlNodePtr curRangeNode = curChildNode->children;
				 while(curRangeNode != NULL)
				 {
					 if(!xmlStrcmp(curRangeNode->name, (const xmlChar *)"DataRecord"))
					 {
						 xmlNodePtr curFieldNode = curRangeNode->children;
						 while(curFieldNode != NULL)
						 {
							 if(!xmlStrcmp(curFieldNode->name, (const xmlChar*)"field"))
							 {
								 szKey = xmlGetProp(curFieldNode,BAD_CAST "name");
								 char* out = 0;
								 if(std::string(mXMLCoding) == "UTF-8")
									 out = ConvertEnc("UTF-8","GB2312", (char*)szKey);
								 std::string sr;
								 if (out)
									 sr = std::string((char*)out);
								 else
									 sr = std::string((char*)szKey);
								 //1:数值属性
								 //2:矢量属性
								 //3:数组属性
								 if(!xmlStrcmp(curFieldNode->children->next->next->next->name, (const xmlChar*)"Quantity"))
								 {
									 mydataarraysname.push_back(sr);
									 mydataarraystype.push_back(1);
								 }
								 if(!xmlStrcmp(curFieldNode->children->next->next->next->name, (const xmlChar*)"Vector"))
								 {
									 mydataarraysname.push_back(sr);
									 mydataarraystype.push_back(2);
								 }
								 if(!xmlStrcmp(curFieldNode->children->next->next->next->name, (const xmlChar*)"DataArray"))
								 {
									 mydataarraysname.push_back(sr);
									 mydataarraystype.push_back(3);
								 }
							 }
							 curFieldNode = curFieldNode->next;
						 }
					 } 
					 curRangeNode = curRangeNode->next;
				 }				 
			 }
			 curChildNode = curChildNode->next;
		 }
	}

	if(mydataarrays.size() != mydataarraysname.size() )
	{
		//返回错误：数据格式有误; 属性名称与值域个数不一致
		// 错误位置：结点的信息
		return false;
	}

	if(ShapeObject)
	for(int i=0; i < mydataarrays.size() && i <mydataarraysname.size(); i++)
	{
		mydataarrays[i]->SetName(mydataarraysname[i].c_str());
	}

	if(ShapeObject)
	for(int i=0; i < mydataarrays.size(); i++)
		dataarrays.push_back(mydataarrays[i]);

	return true;
}


bool GeometryXMLReader::ReadNode(xmlNodePtr geometryNode, gmml::MappedFeature* mappedFeature)
{
	if(!mappedFeature)
		return false;

	xmlNodePtr curNode = geometryNode;
	if(!xmlStrcmp(curNode->name, (const xmlChar*)"Geometry"))
	{
		curNode = curNode->children;
		while(curNode)
		{
			if(!xmlStrcmp(curNode->name, (const xmlChar*)"Shape"))
			{
				xmlNodePtr curShapeNode = curNode->children;
				if(!xmlStrcmp(curShapeNode->name, (const xmlChar*)"text"))
					curShapeNode = curShapeNode->next;
				vtkObject* value = ReadShape(curShapeNode);
				ShapeObject = value;
				mappedFeature->SetShape(value);
			//	value->Delete();
			}
			else if(!xmlStrcmp(curNode->name, (const xmlChar*)"ShapeProperty"))
			{
				ReadProperty(curNode);
			}
			else if(!xmlStrcmp(curNode->name, (const xmlChar*)"PropertyModels"))
			{
				ReadProperty(curNode);
			}
			else if(!xmlStrcmp(curNode->name, (const xmlChar*)"Propertymodels"))
			{
				ReadProperty(curNode);
			}
			curNode = curNode->next;
		}
	}
	return true;
}

vtkObject* GeometryXMLReader::GetReadShape()
{
	return ShapeObject;
}


 