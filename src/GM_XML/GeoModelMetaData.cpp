#include "GeoModelMetaData.h"

using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.08
GeoModelMetaData::GeoModelMetaData(void)
{
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.08
GeoModelMetaData::~GeoModelMetaData(void)
{
}

///@brief  获取属性数目
///@param
///@return 返回属性总数
///@author zhaowei
///@date   2013.11.08
int GeoModelMetaData::GetAttributeValueCount()
{
	int attr_count = attributeValues.size();
	return attr_count;
}

///@brief  获取指定序号的属性
///@param i 序号
///@return 返回指定属性
///@author zhaowei
///@date   2013.11.08
 
AttributeValue GeoModelMetaData::GetAttributeValue(const int& i)
{
	if(i < 0 || i >= attributeValues.size())
	{
		AttributeValue v;
		v.fieldName = "";
		v.fieldValue = "";
		return v;
	}
	AttributeValue attr_value = attributeValues[i];
	return attr_value;
}

///@brief  获取指定属性名的属性
///@param  fieldname 属性名
///@return 返回指定属性
///@author zhaowei
///@date   2013.11.08
AttributeValue GeoModelMetaData::GetAttributeValue(const std::string& fieldname)
{
	for(int i = 0; i < attributeValues.size(); i++)
	{
		if(attributeValues[i].fieldName == fieldname)
		{
			return attributeValues[i];
		}
	}
	AttributeValue v;
	v.fieldName = fieldname;
	v.fieldValue = "";
	return v;
}

///@brief  向AttributeValue容器中添加新的元素
///@param const AttributeValue& value 待添加的新元素
///@return 
///@author zhaowei
///@date   2013.11.08
void GeoModelMetaData::AddAttributeValue(const AttributeValue& value)
{
	attributeValues.push_back(value);
}

void GeoModelMetaData::deleteMembers()
{

}