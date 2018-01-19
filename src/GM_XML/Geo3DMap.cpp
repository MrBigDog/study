#include "Geo3DMap.h"
#include "Geo3DMapMetaData.h"
#include "Geo3DLayer.h"

using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.07
Geo3DMap::Geo3DMap(void)
{
	metaData = new Geo3DMapMetaData;
}

///@brief 析构函数
///@author zhaowei
///@date   2013.11.07
Geo3DMap::~Geo3DMap(void)
{
	deleteMembers();

}

///@brief 获取GeoMap对应的元数据
///@param 
///@return 返回指向Geo3DMapMetaData的指针
///@author zhaowei
///@date   2013.11.01
Geo3DMapMetaData* Geo3DMap::GetGeoMapMetaData()
{
	return metaData;
}


///@brief  设置GeoMap对应的元数据
///@param  value 待设置的元数据
///@return
///@author zhaowei
///@date   2013.11.08
void Geo3DMap::SetGeoMapMetaData(Geo3DMapMetaData* value)
{
	value->setContainer(this);
	metaData = value;
}

///@name 地质图中的图层。
//@{
///@brief  取地质图中所包含的图层的数目。
///@return 返回地质图中所包含图层的数目。
///@author zhaowei
///@date   2013.11.07
int Geo3DMap::GetGeo3DLayerCount()
{
	int layer_count = geo3DLayers.size();
	return layer_count;
}

///@brief  取地质图中的图层。
///@param i 图层在地质图中的索引号（从0开始）。
///@return 指定的图层。所返回指针所指向对象的内存由本对象负责管理。如果指定的图层不存在，则返回0.
///@author zhaowei
///@date   2013.11.07
Geo3DLayer* Geo3DMap::GetLayer(int i)
{
	if(i < 0 || i >= geo3DLayers.size())
		return NULL;
	Geo3DLayer* geo_layer = geo3DLayers[i];
	return geo_layer;
}
//@}

///@brief  获取全部的地质图层
///@param  geo_layers 作为引用参数，保存取出的全部地质图层
///@return 
///@author zhaowei
///@date   2013.11.08
void Geo3DMap::GetAllGeoLayer(std::vector<Geo3DLayer*>& geo_layers)
{
	geo_layers = geo3DLayers;
}

///@brief  取地质图的类型信息。
///@return 返回地质图的类型。
///@author zhaowei
///@date   2013.11.07
std::string Geo3DMap::GetType()
{
	return geoMapType;
}

///@brief  设置地质图类型
///@param  mapType 地质图类型值
///@return
///@author zhaowei
///@date   2013.11.07
void Geo3DMap::SetType(const std::string& mapType)
{
	geoMapType = mapType;
}

///@brief  添加地质图层
///@param  value 待添加的地质图层
///@return
///@author zhaowei
///@date   2013.11.07
void Geo3DMap::AddGeoLayer(Geo3DLayer* value)
{
	value->setContainer(this);
	geo3DLayers.push_back(value);
}

///@brief  获取GeoMap的属性数目
///@param  
///@return 返回GeoMap的属性书目
///@author zhaowei
///@date   2013.11.07
int Geo3DMap::GetAttributeValueCount()
{
	int attribute_count = attributeValues.size();
	return attribute_count;
}

///@brief  获取地质图中第i个属性
///@param  i 序号i（从0开始计数）
///@return 如果序号i有效，返回第i个属性；否则返回属性名称和属性值都为空的属性
///@author zhaowei
///@date   2013.11.07
AttributeValue Geo3DMap::GetAttributeValue(const int& i)
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

///@brief  获取属性名称为fieldname的属性字段
///@param  fieldname 属性名称
///@return 如果属性名称有效，则返回对应的属性；否则返回属性名称和属性值都为空的属性
///@author zhaowei
///@date   2013.11.07
AttributeValue Geo3DMap::GetAttributeValue(const std::string& fieldname)
{
	AttributeValue v;
	if(fieldname.empty())
	{
		v.fieldName = "";
		v.fieldValue = "";
	}
	else
	{
		for(int i = 0; i < attributeValues.size(); i++)
		{
			if(attributeValues[i].fieldName == fieldname)
			{
				v = attributeValues[i];
				break;
			}
		}
	}
	return v;
}

///@brief  向AttributeValue容器中添加新的元素
///@param const AttributeValue& value 待添加的新元素
///@return 
///@author zhaowei
///@date   2013.11.05
void Geo3DMap::AddAttributeValue(const AttributeValue& value)
{
	attributeValues.push_back(value);
}

///@brief  释放管理的内存
///@param  
///@return
///@author zhaowei
///@date   2013.11.13
void Geo3DMap::deleteMembers()
{
	if(metaData)
		delete metaData;
	metaData = NULL;

	for(std::vector<Geo3DLayer*>::size_type i = 0; i < geo3DLayers.size(); i++)
	{
		if(geo3DLayers[i]->getContainer() == this)
		{
			delete geo3DLayers[i];
		}
	}
	geo3DLayers.clear();
}