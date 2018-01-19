#pragma once
#include "xmlwriter.h"
#include "Geo3DStyle.h"
#include <fstream>

namespace se
{
	class GeoSymbolizer;
	class FeatureStyleRule;
	class CoverageStyleRule;
}
namespace gmml
{
	class Geo3DStyle;
}
class GMXMLFILEIO_EXPORT Geo3DStyleXMLWriter :
	public XMLWriter
{
public:
	///@brief 构造函数
	///@author zhaowei
	///@date 2014.03.18
	Geo3DStyleXMLWriter(void);

	///@brief 析构函数
	///@author zhaowei
	///@date 2014.03.18
	~Geo3DStyleXMLWriter(void);

	///@brief 将Style信息写入文件
	///@param style 指向待写入Style的指针
	///@param xml_stream 写出文件流指针
	///@author zhaowei
	///@date 2014.03.18
	void WriteStyle(gmml::Geo3DStyle* style, std::ofstream& xml_stream);

	///@brief 将Symbolizer信息写入文件
	///@param rule 包含Symbolizer数据的指向rule的指针
	///@param xml_stream 写出文件流
	///@author zhaowei
	///@date 2014.03.18
	void WirteSymbolizer(se::FeatureStyleRule* rule, std::ofstream& xml_stream);
	void WirteCoverageSymbolizer(se::CoverageStyleRule* rule, std::ofstream& xml_stream);
};

