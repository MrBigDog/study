#pragma once

#include "AbstractGML.h"

namespace gml
{

class GEOXML_EXPORT AbstractFeature : public AbstractGML
{
public:
	///@brief  构造函数
	///@author zhaowei
	///@date   2013.11.13
	AbstractFeature(void);

	///@brief  析构函数
	///@author zhaowei
	///@date   2013.11.13
	~AbstractFeature(void);

	///@brief  获取类名称
	///@param 
	///@return 返回类名称
	///@author zhaowei
	///@date   2013.11.13
	virtual std::string GetClassName(){return "AbstractFeature";};

	///@brief  获取属性数目
	///@param 
	///@return 返回属性数目
	///@author zhaowei
	///@date   2013.11.13
	int GetAttributeValueCount();

	///@brief  获取序号为i的属性
	///@param  i 序号
	///@return 返回属性
	///@author zhaowei
	///@date   2013.11.13
	AttributeValue GetAttributeValue(const int& i);

	bool GetAttributeValue(const int& i, AttributeValue*& pOut);

	///@brief  获取属性名为fieldname的属性
	///@param  fieldname 属性名称
	///@return 如果属性名称为空，返回属性名称和属性值为空的对象；否则返回相应的属性
	///@author zhaowei
	///@date   2013.11.13
	AttributeValue GetAttributeValue(std::string fieldname);

	///@brief  向AttributeValue容器中添加新的元素
	///@param  const AttributeValue& value 待添加的新元素
	///@return 
	///@author zhaowei
	///@date   2013.11.13
	void AddAttributeValue(const AttributeValue& value);

protected:
	std::vector<AttributeValue>  attributeValues;//feilds
};

}