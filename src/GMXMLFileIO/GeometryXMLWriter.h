#pragma once
#include "xmlwriter.h"

class vtkObject;
class GMPolyData;
class GMUnstructuredGrid;
class GMStructuredPoints;
class GMCornerPointGrid;
class GMXMLFILEIO_EXPORT GeometryXMLWriter : public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.19
	GeometryXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.03.19
	~GeometryXMLWriter(void);

	static int BinaryExport;	//是否作为二进制输出的标识符

	///@brief 将Geometry信息写入文件
	///@param shape 几何信息
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.19
	void WriteGeometry(vtkObject* shape, std::ostream& xml_stream);

	void WritePoint(GMPolyData* poly_data,std::ostream& xml_stream);

	//写多点数据
	//2014-11-26
	//yhj
	void WriteMultiPoint(GMPolyData* poly_data,std::ostream& xml_stream);

	void WritePointBinary(GMPolyData* poly_data,std::ostream& xml_stream);

	void WriteLinestring(GMPolyData* poly_data,std::ostream& xml_stream);

	void WriteMultiLinestring(GMPolyData* poly_data,std::ostream& xml_stream);

	void WriteLinestringBinary(GMPolyData* poly_data,std::ostream& xml_stream);

	void WriteTIN(GMPolyData* poly_data,std::ostream& xml_stream);

	void WriteTINBinary(GMPolyData* poly_data,std::ostream& xml_stream);

	void WriteGeoVolume(GMUnstructuredGrid* grid, std::ostream& xml_stream);

	void WriteGeoVolumeBinary(GMUnstructuredGrid* grid, std::ostream& xml_stream);

	void WriteStructedPoints(GMStructuredPoints* structed_points, std::ostream& xml_stream);

	void WriteVertex(GMPolyData* poly_data, std::ostream& xml_stream);

	void WriteGeoCornerPointGrid(GMCornerPointGrid* poly_data, std::ostream& xml_stream);
	void WriteGeoCornerPointGridBinary(GMCornerPointGrid* poly_data, std::ostream& xml_stream);
};

