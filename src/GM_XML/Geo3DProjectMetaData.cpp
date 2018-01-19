#include "Geo3DProjectMetaData.h"

using namespace  gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.08
Geo3DProjectMetaData::Geo3DProjectMetaData(void)
{

}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.08
Geo3DProjectMetaData::~Geo3DProjectMetaData(void)
{

}

///@brief  获取属性数目
///@param 
///@return 返回属性总数
///@author zhaowei
///@date   2013.11.08
int Geo3DProjectMetaData::GetAttributeValueCount()
{
	int attr_count = attributeValues.size();
	return attr_count;
}

///@brief  获取指定序号的属性
///@param  i 序号
///@return 返回指定属性
///@author zhaowei
///@date   2013.11.08
AttributeValue Geo3DProjectMetaData::GetAttributeValue(const int& i)
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
AttributeValue Geo3DProjectMetaData::GetAttributeValue(const std::string& fieldname)
{
	for(int i = 0; i < attributeValues.size(); i++)
	{
		if(attributeValues[i].fieldName == fieldname)
		{
			return attributeValues[i];
		}
	}
	/// 如果没有该属性，则返回fieldName为传入参数、fieldValue为空的AttributeValue
	AttributeValue r;
	r.fieldName = fieldname;
	r.fieldValue = "";
	return r;
}

///@brief  向AttributeValue容器中添加新的元素
///@param const AttributeValue& value 待添加的新元素
///@return 
///@author zhaowei
///@date   2013.11.08
void Geo3DProjectMetaData::AddAttributeValue(const AttributeValue& value)
{
	attributeValues.push_back(value);
}