#ifndef Geo3DProjectMetaData_h__
#define Geo3DProjectMetaData_h__

#include "AbstractGML.h"

namespace gmml
{
	class GEOXML_EXPORT Geo3DProjectMetaData : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.08
		Geo3DProjectMetaData(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.08
		~Geo3DProjectMetaData(void);

		///@brief  获取属性数目
		///@param
		///@return 返回属性总数
		///@author zhaowei
		///@date   2013.11.08
		int GetAttributeValueCount();

		///@brief  获取指定序号的属性
		///@param i 序号
		///@return 返回指定属性
		///@author zhaowei
		///@date   2013.11.08
		AttributeValue GetAttributeValue(const int& i);

		///@brief  获取指定属性名的属性
		///@param  fieldname 属性名
		///@return 返回指定属性
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
		void deleteMembers() { ; };
	protected:
		std::vector<AttributeValue>  attributeValues;
	};

}

#endif // Geo3DProjectMetaData_h__
