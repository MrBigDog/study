#ifndef FeatureClass_h__
#define FeatureClass_h__

#include "AbstractFeature.h" 

namespace gmml
{
	class GeologicFeature;
	class GEOXML_EXPORT GeologicFeatureClass : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.08
		GeologicFeatureClass(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.08
		~GeologicFeatureClass(void);

		///@brief  获取该类名称
		///@param
		///@return 返回该类名称
		///@author zhaowei
		///@date   2013.11.08
		virtual std::string GetClassName() { return "GeologicFeatureClass"; };

		///@brief  获取指定序号的Schema项
		///@param i 序号
		///@return 返回指定序号的Schema项
		///@author zhaowei
		///@date   2013.11.08
		AttributeValue& GetFeatureClassSchema(const int& i);

		///@brief 获取包含的Schema数目
		///@return 返回包含的Schema总数
		///@author zhaowei
		///@date 2013.12.03
		int GetFeatureClassSchemaCount();

		///@brief  获取指定名称的Schema项
		///@param fieldname 名称
		///@return 返回相应的Schema项
		///@author zhaowei
		///@date   2013.11.08
		AttributeValue GetFeatureClassSchema(const std::string& field_name);

		///@brief  获取全部的Schema
		///@param  feature_class_schemas 作为引用变量，保存传出的所有Schema
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void GetAllFeatureClassSchema(std::vector<AttributeValue>& feature_class_schemas);

		///@brief  添加新的Schema项
		///@param  value 待添加的Schema项
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void AddFeatureClassSchema(const AttributeValue& value);

		///@brief  获取指定序号的GeologicFeature
		///@param  i 序号
		///@return 返回第i个GeologicFeature
		///@author zhaowei
		///@date   2013.11.08
		GeologicFeature* GetGeologicFeature(const int& i);

		///@brief 获取包含的GeoFeature数目
		///@return 返回包含的GeoFeature总数
		///@author zhaowei
		///@date 2013.12.03
		int GetGeologicFeatureCount();

		///@brief  获取全部的GeologicFeature
		///@param  geo_features 作为引用变量，保存取出的全部GeologicFeature
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void GetAllGeologicFeature(std::vector<GeologicFeature*>& geo_features);

		///@brief  添加GeologicFeature
		///@param  value 待添加的GeologicFeature
		///@return
		///@author zhaowei
		///@date   2013.11.08
		void AddGeologicFeature(GeologicFeature* value);

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();
	protected:
		std::vector<AttributeValue>  schema_;
		std::vector<GeologicFeature*>  features;
	};
}

#endif // FeatureClass_h__
