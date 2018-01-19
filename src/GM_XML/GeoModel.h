#ifndef GeoModel_h__
#define GeoModel_h__

#include "AbstractGML.h"

namespace gmml
{
	class GeoModelMetaData;
	class FeatureRelation;
	class GeologicFeatureClass;
	class GEOXML_EXPORT GeoModel : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.06
		GeoModel(void);

		///@brief 析构函数
		///@author zhaowei
		///@date   2013.11.06
		~GeoModel(void);

		///@brief  获取该类的名称
		///@param 
		///@return 返回该类的名称
		///@author zhaowei
		///@date   2013.11.06
		std::string GetClassName() { return "GeoModel"; };

		///@brief  获取GeoModel对应的元数据
		///@param 
		///@return 返回指向GeoModelMetaData的指针
		///@author zhaowei
		///@date   2013.11.08
		GeoModelMetaData* GetGeoModelMetaData();

		///@brief  设置GeoModel对应的元数据
		///@param value 待设置的元数据
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void SetGeoModelMetaData(GeoModelMetaData* value);

		///@brief  获取模型类型
		///@param 
		///@return 返回该模型的类型
		///@author zhaowei
		///@date   2013.11.07
		std::string GetType();

		///@brief  设置模型类型
		///@param  value 待设置的模型类型
		///@return
		///@author zhaowei
		///@date   2013.11.08
		void SetType(const std::string& value);

		///@brief  获取属性数目
		///@param 
		///@return 返回属性数目
		///@author zhaowei
		///@date   2013.11.06
		int GetAttributeValueCount();

		///@brief  获取序号为i的属性
		///@param  i 序号
		///@return 返回属性
		///@author zhaowei
		///@date   2013.11.06
		AttributeValue GetAttributeValue(const int& i);

		///@brief  获取属性名为fieldname的属性
		///@param  fieldname 属性名称
		///@return 返回属性
		///@author zhaowei
		///@date   2013.11.06
		AttributeValue GetAttributeValue(const std::string& fieldname);

		///@brief  向AttributeValue容器中添加新的元素
		///@param  const AttributeValue& value 待添加的新元素
		///@return 
		///@author zhaowei
		///@date   2013.11.05
		void AddAttributeValue(const AttributeValue& value);

		///@brief 获取Model包含的FeatureClass数目
		///@return 返回FeatureClass数目
		///@author wang
		///@date 2013.11.26
		int GetGeoFeatureClassCount();

		///@brief  获取指定序号的FeatureClass
		///@param  i 序号
		///@return 返回指向FeatureClass的指针 
		///@author zhaowei
		///@date   2013.11.08
		GeologicFeatureClass* GetGeoFeatureClass(const int& i);

		///@brief 获取指定ID的FeatureClass
		///@param fcID FeatureClass的ID
		///@return 返回FeatureClass指针
		///@author wang
		///@date 2013.11.26
		GeologicFeatureClass* GetGeoFeatureClass(const std::string& fcID);

		///@brief  获取全部的FeatureClass
		///@param  geo_feature_class 作为引用参数，保存取出的FeatureClass
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void GetAllFeatureClass(std::vector<GeologicFeatureClass*>& geo_feature_class);

		///@brief  添加新的FeatureClass
		///@param  value 待添加的FeatureClass
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void AddFeatureClass(GeologicFeatureClass* value);

		///@brief  获取指定序号的FeatureRelation
		///@param  i 序号
		///@return 返回指向FeatureRelation的指针
		///@author zhaowei
		///@date   2013.11.08
		FeatureRelation* GetGeoFeatureRelation(const int& i);

		///@brief 获取Model中的FeatureRelation数目
		///@return 返回FeatureRelation数目
		///@author wang
		///@date 2013.11.26
		int GetGeoFeatureRelationCount();

		///@brief  获取全部的FeatureRelation
		///@param  geo_feature_relation 作为引用参数，保存取出的FeatureRelation
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void GetAllFeatureRelation(std::vector<FeatureRelation*>& geo_feature_relation);

		///@brief  添加新的FeatureRelation
		///@param  value 待添加的FeatureRelation
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void AddFeatureRelation(FeatureRelation* value);

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();

		///@brief 添加Model中所有Feature的相关FeatureRelation
		///@author wang
		///@date 2013.11.26
		void UpdateRelations();
	protected:
		/// Model的自定义属性数组
		std::vector<AttributeValue>          attributeValues;
		/// Model包含的FeatureClass数组
		std::vector<GeologicFeatureClass*>   geoFeatureClasses;
		/// Model包含的FeatureRelation数组
		std::vector<FeatureRelation*>        featureRelations;
		/// Model对应的元数据
		GeoModelMetaData*					 metaData;
		/// Model的类型
		std::string							 geoModelType;
	};

}


#endif // GeoModel_h__
