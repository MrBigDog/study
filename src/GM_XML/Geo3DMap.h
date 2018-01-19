#ifndef Geo3DMap_h__
#define Geo3DMap_h__

#include "AbstractGML.h"

namespace gmml
{

	class Geo3DMapMetaData;
	class Geo3DLayer;
	///@brief 地质图
	class GEOXML_EXPORT Geo3DMap : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.07
		Geo3DMap(void);

		///@brief 析构函数
		///@author zhaowei
		///@date   2013.11.07
		~Geo3DMap(void);

		///@brief  获取该类的名字
		///@param 
		///@return 返回该类的名称
		///@author zhaowei
		///@date   2013.11.07
		virtual std::string GetClassName() { return "Geo3DMap"; };

	public:


		///@brief  获取GeoMap对应的元数据
		///@param
		///@return 返回指向Geo3DMapMetaData的指针
		///@author zhaowei
		///@date   2013.11.01
		Geo3DMapMetaData* GetGeoMapMetaData();

		///@brief  设置GeoMap对应的元数据
		///@param  value 待设置的元数据
		///@return
		///@author zhaowei
		///@date   2013.11.08
		void SetGeoMapMetaData(Geo3DMapMetaData* value);

		///@name 地质图中的图层。
		//@{
		///@brief  取地质图中所包含的图层的数目。
		///@return 返回地质图中所包含图层的数目。
		///@author zhaowei
		///@date   2013.11.07
		int GetGeo3DLayerCount();

		///@brief  取地质图中的图层。
		///@param  i 图层在地质图中的索引号（从0开始）。
		///@return 指定的图层。所返回指针所指向对象的内存由本对象负责管理。如果指定的图层不存在，则返回0.
		///@author zhaowei
		///@date   2013.11.07
		Geo3DLayer* GetLayer(int i);
		//@}

		///@brief  获取全部的地质图层
		///@param geo_layers 作为引用参数，保存取出的全部地质图层
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void GetAllGeoLayer(std::vector<Geo3DLayer*>& geo_layers);

		///@brief  取地质图的类型信息。
		///@return 返回地质图的类型。
		///@author zhaowei
		///@date   2013.11.07
		std::string GetType();

		///@brief  设置地质图类型
		///@param mapType 地质图类型值
		///@return
		///@author zhaowei
		///@date   2013.11.07
		void SetType(const std::string& mapType);

		///@brief  添加地质图层
		///@param  value 待添加的地质图层
		///@return
		///@author zhaowei
		///@date   2013.11.07
		void AddGeoLayer(Geo3DLayer* value);

		///@brief  获取GeoMap的属性数目
		///@param  
		///@return 返回GeoMap的属性书目
		///@author zhaowei
		///@date   2013.11.07
		int GetAttributeValueCount();

		///@brief  获取地质图中第i个属性
		///@param i 序号i（从0开始计数）
		///@return 如果序号i有效，返回第i个属性；否则返回属性名称和属性值都为空的属性
		///@author zhaowei
		///@date   2013.11.07
		AttributeValue GetAttributeValue(const int& i);

		///@brief  获取属性名称为fieldname的属性字段
		///@param  fieldname 属性名称
		///@return 如果属性名称有效，则返回对应的属性；否则返回属性名称和属性值都为空的属性
		///@author zhaowei
		///@date   2013.11.07
		AttributeValue GetAttributeValue(const std::string& fieldname);

		///@brief  向AttributeValue容器中添加新的元素
		///@param const AttributeValue& value 待添加的新元素
		///@return 
		///@author zhaowei
		///@date   2013.11.05
		void AddAttributeValue(const AttributeValue& value);

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();
	protected:
		std::vector<AttributeValue>  attributeValues;
		std::vector<Geo3DLayer*>     geo3DLayers;
		gmml::Geo3DMapMetaData*      metaData;
		std::string                  geoMapType;
	};

}
#endif // Geo3DMap_h__