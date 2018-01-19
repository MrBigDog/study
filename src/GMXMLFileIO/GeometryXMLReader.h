#pragma once
#include "xmlreader.h"
#include <vector>
#include "MappedFeature.h"

class vtkObject;
class GMPolyData;
class GMUnstructuredGrid;
class vtkPointSet;
class GMStructuredPoints;
class vtkDataArray;
class GMCornerPointGrid;

class GMXMLFILEIO_EXPORT GeometryXMLReader :
	public XMLReader
{
public:
	GeometryXMLReader(void);
	~GeometryXMLReader(void);
	//gml::AbstractGML* GetShape();
	static int BinaryExport;	//是否作为二进制读的标识符
	vtkObject* GetReadShape();


	bool ReadNode(xmlNodePtr geometryNode, gmml::MappedFeature* mappedFeature);
	virtual vtkObject* ReadShape(xmlNode* geometryNode);
	vtkObject* ReadPolyDataTIN(xmlNode* geometryNode);
	vtkObject* ReadPolyDataTEN(xmlNode* geometryNode);
	vtkDataArray*  GeometryXMLReader::ReadDiscreteCoverageGMML(xmlNode* geometryNode);
	vtkDataArray*  GeometryXMLReader::ReadDataArray(xmlNode* arrayNode);
	///@brief  读取<LineString>标签
	///@param  geometryNode <LineString>标签的xml结点指针
	///@return 将LineString保存为vtkObject返回
	///@author zhaowei
	///@date   2013.11.19
	vtkObject* ReadPolyDataLine(xmlNode* geometryNode);
	vtkObject* ReadPolyDataPoints(xmlNode* geometryNode);
	vtkObject* ReadGeoVolume(xmlNode* geometryNode);
	vtkObject* ReadGeoGrid(xmlNode* geometryNode);
	vtkObject* ReadGeoCornerPointGrid(xmlNode* geometryNode);

	vtkDataArray* ReadValueArray(xmlNode* arrayNode);

	///@brief  读取<DiscreteCoverage>标签下的内容
	///@param  dataarrays 保存<ValueArray>标签下的内容
	///@param  dataarraytypes 保存ValueArray的类型编号
	///@param  dataarraysname 保存ValueArray对应的类型名称
	///@return 如果RangeSet中的ValueArray值和RangeType中规定的属性字段一致，返回true；否则返回false
	///@author zhaowei
	///@date   2013.11.18
	bool ReadDiscreteCoverage(xmlNode* geometryNode,std::vector<vtkDataArray*> &dataarrays,std::vector<int> &dataarraytypes, std::vector<std::string>& dataarraysname);

	///@brief  读取<ShapeProperty>标签
	///@param  arrayNode 指向<ShapeProperty>标签的xml指针
	///@author zhaowei
	///@date   2013.11.18
	void ReadProperty(xmlNode* arrayNode);
		 
protected:
	bool ReadRectifiedGrid(GMStructuredPoints*,xmlNode* geometryNode);
	bool ReadPillars(GMCornerPointGrid*,xmlNode* geometryNode);
	bool ReadCCells(GMCornerPointGrid*,xmlNode* geometryNode);
	bool ReadVertices(vtkPointSet*,xmlNode*);
	bool ReadTriangles(GMPolyData*,xmlNode*);
	bool ReadCells(GMUnstructuredGrid*,xmlNode*);

	 
		vtkObject* ShapeObject;
	 

};

