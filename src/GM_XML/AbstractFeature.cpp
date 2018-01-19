#include "AbstractFeature.h"
 
using namespace gml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.13
AbstractFeature::AbstractFeature(void)
{
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.13
AbstractFeature::~AbstractFeature(void)
{
}

///@brief  获取属性数目
///@param
///@return 返回属性数目
///@author zhaowei
///@date   2013.11.13
int AbstractFeature::GetAttributeValueCount()
{
	int attr_count = attributeValues.size();
	return attr_count;
}

///@brief  获取序号为i的属性
///@param i 序号
///@return 返回属性
///@author zhaowei
///@date   2013.11.13
AttributeValue AbstractFeature::GetAttributeValue(const int& i)
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

bool AbstractFeature::GetAttributeValue(const int& i, AttributeValue*& pOut)
{
	pOut = NULL;
	if (i < 0 || i >= attributeValues.size())
	{
		return false;
	}
	pOut = &attributeValues[i];
	return true;
}

///@brief  获取属性名为fieldname的属性
///@param fieldname 属性名称
///@return 如果属性名称为空，返回属性名称和属性值为空的对象；否则返回相应的属性
///@author zhaowei
///@date   2013.11.13
AttributeValue AbstractFeature::GetAttributeValue(std::string fieldname)
{
	if(fieldname.empty())
	{
		AttributeValue v;
		v.fieldName = "";
		v.fieldValue = "";
		return v;
	}
	for(int i = 0; i < attributeValues.size(); i++)
	{
		if(attributeValues[i].fieldName == fieldname)
		{
			return attributeValues[i];
		}
	}
}

///@brief  向AttributeValue容器中添加新的元素
///@param const AttributeValue& value 待添加的新元素
///@return 
///@author zhaowei
///@date   2013.11.13
void AbstractFeature::AddAttributeValue(const AttributeValue& value)
{
	attributeValues.push_back(value);
}