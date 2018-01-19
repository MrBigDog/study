#ifndef Geo3DLayer_h__
#define Geo3DLayer_h__

#include "AbstractGML.h"

namespace gmml
{
	class GeologicFeatureClass;
	class Geo3DStyle;
	class GEOXML_EXPORT Geo3DLayer : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.07
		Geo3DLayer(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.07
		~Geo3DLayer(void);

		///@brief  获取该类的名称
		///@param  
		///@return 返回该类的名称
		///@author zhaowei
		///@date   2013.11.07
		virtual std::string GetClassName() { return "Geo3DLayer"; };

	public:
		///@brief  获取Geo3DLayer对应要素类
		///@param  
		///@return 返回指向要素类的指针
		///@author zhaowei
		///@date   2013.11.07
		GeologicFeatureClass* GetFeatureClass();

		///@brief  获取可视化参数
		///@param  
		///@return 返回指向可视化参数GeoStyle的指针
		///@author zhaowei
		///@date   2013.11.07
		Geo3DStyle* GetStyle();

		///@brief  设置Geo3DLayer对应的要素类
		///@param  value 待设置的要素类
		///@return 
		///@author zhaowei
		///@date   2013.11.07
		void SetFeatureClass(GeologicFeatureClass* value);

		///@brief  设置Geo3DLayer对应的显示参数
		///@param  value 待设置的显示参数
		///@return 
		///@author zhaowei
		///@date   2013.11.07
		void SetStyle(Geo3DStyle* value);

		///@brief  获取属性数目
		///@param 
		///@return 返回属性数目
		///@author zhaowei
		///@date   2013.11.08
		int GetAttributeValueCount();

		///@brief  获取序号为i的属性
		///@param  i 序号
		///@return 返回属性
		///@author zhaowei
		///@date   2013.11.08
		AttributeValue GetAttributeValue(const int& i);

		///@brief  获取属性名为fieldname的属性
		///@param  fieldname 属性名称
		///@return 如果属性名称为空，返回属性名称和属性值为空的对象；否则返回相应的属性
		///@author zhaowei
		///@date   2013.11.08
		AttributeValue GetAttributeValue(const std::string& fieldname);

		///@brief  向AttributeValue容器中添加新的元素
		///@param  const AttributeValue& value 待添加的新元素
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void AddAttributeValue(const AttributeValue& value);

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();
	protected:
		GeologicFeatureClass*		featureClass;
		Geo3DStyle*					geoStyle;
		std::vector<AttributeValue> attributeValues;
	};

}

#endif // Geo3DLayer_h__
