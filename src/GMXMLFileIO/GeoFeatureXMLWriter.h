#pragma once
#include "xmlwriter.h"
#include <string>
#include "vtkPointData.h"
#include "../vtkExtending/GMPolyData.h"
#include "../vtkExtending/GMUnstructuredGrid.h"
#include "../vtkExtending/GMStructuredPoints.h"
#include "vtkCellArray.h"
#include "vtkMath.h"
#include "vtkPoints.h"
#include "vtkPointData.h"
#include "vtkCellData.h"
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"

#include "vtkCharArray.h"
#include "vtkLongArray.h"
#include "vtkIntArray.h"
//#include "vtkPolyDataNormals.h"

namespace gmml
{
	class GeologicFeature;
	class MappedFeature;
}
class GMXMLFILEIO_EXPORT GeoFeatureXMLWriter :	public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.19
	GeoFeatureXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.04.19
	~GeoFeatureXMLWriter(void);

	///@brief 将GeoFeature写入文件
	///@param feature 指向Feature的指针
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.19	
	void WriteFeature(gmml::GeologicFeature* feature, std::ostream& xml_stream);

	///@brief 将MappedFeature写入文件
	///@param mapped_feature 指向MappedFeature的指针
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.19
	void WriteMappedFeature(gmml::MappedFeature* mapped_feature, std::ostream& xml_stream);

	///@brief 将MappedPropery写入文件
	///@param feature 指向MappedFeature的指针	
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.19
	void WriteMappedProperty(gmml::MappedFeature* mapped_feature, std::ostream& xml_stream);

	///@brief 将DiscreteCoverage写入文件
	///@param position_type 属性场的位置
	///@param shape_property 属性信息
	///@param xml_stream 写出文件流
	///@param geoid 属性场编号
	///@author zhaowei
	///@date 2014.03.19
	void WriteDiscreteCoverage(int position_type, vtkObject* shape_property, std::ostream& xml_stream, std::string geoid);
};

