#include "Geo3Dlayer.h"
#include "FeatureClass.h"
#include "Geo3DStyle.h"

using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.07
Geo3DLayer::Geo3DLayer(void)
{
	featureClass = NULL;
	geoStyle = NULL;
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.07
Geo3DLayer::~Geo3DLayer(void)
{
	deleteMembers();
}

///@brief  获取Geo3DLayer对应要素类
///@param 
///@return 返回指向要素类的指针
///@author zhaowei
///@date   2013.11.07
GeologicFeatureClass* Geo3DLayer::GetFeatureClass()
{
	return featureClass;
}

///@brief  获取可视化参数
///@param 
///@return 返回指向可视化参数GeoStyle的指针
///@author zhaowei
///@date   2013.11.07
Geo3DStyle* Geo3DLayer::GetStyle()
{
	return geoStyle;
}

///@brief  释放管理的内存
///@param  
///@return
///@author zhaowei
///@date   2013.11.13
void Geo3DLayer::deleteMembers()
{
   if(featureClass && featureClass->getContainer() == this)
	   delete featureClass;

   if(geoStyle && geoStyle->getContainer() == this)
	   delete geoStyle;
}
///@brief  设置Geo3DLayer对应的要素类
///@param  value 待设置的要素类
///@return 
///@author zhaowei
///@date   2013.11.07
void Geo3DLayer::SetFeatureClass(GeologicFeatureClass* value)
{
	if(featureClass && featureClass->getContainer() == this)
		delete featureClass;

	if(value->getContainer() == NULL)
		value->setContainer(this);

	featureClass = value;

}

///@brief  设置Geo3DLayer对应的显示参数
///@param  value 待设置的显示参数
///@return 
///@author zhaowei
///@date   2013.11.07
void Geo3DLayer::SetStyle(Geo3DStyle* value)
{	
	if(geoStyle && geoStyle->getContainer() == this)
		delete geoStyle;	
	
	if(value)
	if(value->getContainer() == NULL)
		value->setContainer(this);

	geoStyle = value;
}

///@brief  获取属性数目
///@param 
///@return 返回属性数目
///@author zhaowei
///@date   2013.11.08
int Geo3DLayer::GetAttributeValueCount()
{
	int attr_count = attributeValues.size();
	return attr_count;
}

///@brief  获取序号为i的属性
///@param  i 序号
///@return 返回属性
///@author zhaowei
///@date   2013.11.08
AttributeValue Geo3DLayer::GetAttributeValue(const int& i)
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

///@brief  获取属性名为fieldname的属性
///@param  fieldname 属性名称
///@return 如果属性名称为空，返回属性名称和属性值为空的对象；否则返回相应的属性
///@author zhaowei
///@date   2013.11.08
AttributeValue Geo3DLayer::GetAttributeValue(const std::string& fieldname)
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
///@param  const AttributeValue& value 待添加的新元素
///@return 
///@author zhaowei
///@date   2013.11.08
void Geo3DLayer::AddAttributeValue(const AttributeValue& value)
{
	attributeValues.push_back(value);
}