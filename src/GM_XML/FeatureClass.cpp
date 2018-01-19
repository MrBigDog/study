#include "FeatureClass.h"
#include "GeologicFeature.h"

using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@2013.11.08
GeologicFeatureClass::GeologicFeatureClass(void)
{

}

///@brief  析构函数
///@author zhaowei
///@2013.11.08
GeologicFeatureClass::~GeologicFeatureClass(void)
{

}
int GeologicFeatureClass:: GetFeatureClassSchemaCount()
{
	return schema_.size();
}
 
int GeologicFeatureClass:: GetGeologicFeatureCount()
{
	return features.size();
}
///@brief  获取指定序号的Schema项
///@param  i 序号
///@return 返回指定序号的Schema项
///@author zhaowei
///@date   2013.11.08
AttributeValue& GeologicFeatureClass::GetFeatureClassSchema(const int& i)
{
	if(i < 0 || i >= schema_.size())
	{
		AttributeValue v;
		v.fieldName = "";
		v.fieldValue = "";
		v.fieldType = "";
		return v;
	}
	return schema_[i];
}

///@brief  获取指定名称的Schema项
///@param fieldname 名称
///@return 返回相应的Schema项
///@author zhaowei
///@date   2013.11.08
AttributeValue GeologicFeatureClass::GetFeatureClassSchema(const std::string& field_name)
{
	if(field_name.empty())
	{
		AttributeValue v;
		v.fieldName = "";
		v.fieldValue = "";
		v.fieldType = "";
		return v;
	}
	for(int i = 0; i < schema_.size(); i++)
	{
		if(schema_[i].fieldName == field_name)
			return schema_[i];
	}
}

///@brief  获取全部的Schema
///@param  feature_class_schemas 作为引用变量，保存传出的所有Schema
///@return 
///@author zhaowei
///@date   2013.11.08
void GeologicFeatureClass::GetAllFeatureClassSchema(std::vector<AttributeValue>& feature_class_schemas)
{
	feature_class_schemas = schema_;
}

///@brief  添加新的Schema项
///@param  value 待添加的Schema项
///@return 
///@author zhaowei
///@date   2013.11.08
void GeologicFeatureClass::AddFeatureClassSchema(const AttributeValue& value)
{
	schema_.push_back(value);
}

///@brief  获取指定序号的GeologicFeature
///@param i 序号
///@return 返回第i个GeologicFeature
///@author zhaowei
///@date   2013.11.08
GeologicFeature* GeologicFeatureClass::GetGeologicFeature(const int& i)
{
	if(i < 0 || i >= features.size())
		return NULL;
	return features[i];
}

///@brief  获取全部的GeologicFeature
///@param  geo_features 作为引用变量，保存取出的全部GeologicFeature
///@return 
///@author zhaowei
///@date   2013.11.08
void GeologicFeatureClass::GetAllGeologicFeature(std::vector<GeologicFeature*>& geo_features)
{
	geo_features = features;
}

///@brief  添加GeologicFeature
///@param  value 待添加的GeologicFeature
///@return
///@author zhaowei
///@date   2013.11.08
void GeologicFeatureClass::AddGeologicFeature(GeologicFeature* value)
{
	value->setContainer(this);
	features.push_back(value);
}

///@brief  释放管理的内存
///@param  
///@return
///@author zhaowei
///@date   2013.11.13
void GeologicFeatureClass::deleteMembers()
{
	for(std::vector<GeologicFeature*>::size_type i = 0; i < features.size(); i++)
	{
		if(features[i]->getContainer() == this)
		{
			delete features[i];
		}
	}
	features.clear();
}