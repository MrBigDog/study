#ifndef Geo3DStyle_h__
#define Geo3DStyle_h__



#include "AbstractGML.h"
#include "Featurestyle.h"

namespace se
{
	class FeatureType3DStyle;
	class CoverageType3DStyle;
}

namespace gmml
{
	class GEOXML_EXPORT Geo3DStyle : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.14
		Geo3DStyle(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.14
		~Geo3DStyle(void);

		///@brief  获取类名称
		///@param
		///@return 类的名称
		///@author zhaowei
		///@date   2013.11.14
		virtual std::string GetClassName() { return "Geo3DStyle"; };

		///@brief  获取FeatureTypeStyle
		///@param  i FeatureTypeStyle的序号
		///@return 返回FeatureTypeStyle
		///@author zhaowei
		///@date   2013.11.14
		se::FeatureType3DStyle* GetFeatureTypeStyle(int i);

		///@brief 获取FeatureTypeStyle的数目
		///@return 返回FeatureTypeStyle的数目
		///@author wang
		///@date 2013.11
		int GetFeatureTypeStyleCount();

		///@brief  设置FeatureTypeStyle
		///@param  value 待设置的FeatureTypeStyle
		///@return
		///@author zhaowei
		///@date   2013.11.14
		void SetFeatureTypeStyle(se::FeatureType3DStyle* value);


		///@brief  获取类名称

	///@brief  获取CoverageTypeStyle
	///@param  i CoverageTypeStyle的序号
	///@return 返回CoverageTypeStyle
	///@author zhaowei
	///@date   2013.11.14
		se::CoverageType3DStyle* GetCoverageTypeStyle(int i);

		///@brief 获取CoverageTypeStyle的数目
		///@return 返回CoverageTypeStyle的数目
		///@author wang
		///@date 2013.11
		int GetCoverageTypeStyleCount();

		///@brief  设置CoverageTypeStyle
		///@param  value 待设置的CoverageTypeStyle
		///@return
		///@author zhaowei
		///@date   2013.11.14
		void SetCoverageTypeStyle(se::CoverageType3DStyle* value);

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();
	private:
		std::vector<se::FeatureType3DStyle*> featureStyles_;
		std::vector<se::CoverageType3DStyle*> coverageStyles_;
	};

}

#endif // Geo3DStyle_h__
