#ifndef SpatialRelation_h__
#define SpatialRelation_h__

#include "FeatureRelation.h"

namespace gmml
{
	class GEOXML_EXPORT SpatialRelation : public FeatureRelation
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.13
		SpatialRelation(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.13
		~SpatialRelation(void);

		///@brief  获取该类的名称
		///@param
		///@return 返回类的名字
		///@author zhaowei
		///@date   2013.11.13
		virtual std::string GetClassName() { return "SpatialRelation"; };

	private:

	};
}

#endif // SpatialRelation_h__
