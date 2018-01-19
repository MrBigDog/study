#include "Geo3DProject.h"
#include "Geo3DProjectMetaData.h"
#include "Geo3DMap.h"
#include "GeoModel.h"
#include "Geo3DLayer.h"
#include "GeoModel.h"
#include "featureclass.h"
#include "GeoSceneStyle.h"

using namespace gmml;

//////////////////////////////////////////////////////////////////////////////
///@brief  构造函数
///@author zhaowei
///@date   2013.11.06
Geo3DProject::Geo3DProject(void)
{
	projectBoundary = NULL;
	pointCount = 0;
	metaData = new Geo3DProjectMetaData;
	metaData->setContainer(this);

	sceneStyle = new GeoSceneStyle;          ;
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.06
Geo3DProject::~Geo3DProject(void)
{
	if(sceneStyle)
		delete sceneStyle;
	deleteMembers();
}
 GeoSceneStyle*           Geo3DProject::GetSceneStyle()
 {
	 return sceneStyle;
 }
void Geo3DProject::setDescription(std::string d)
{
	projDescription = d;
}
std::string Geo3DProject::getDescription()
{
	return projDescription;
}

///@brief  释放管理的内存
///@param  
///@return
///@author zhaowei
///@date   2013.11.13
void Geo3DProject::deleteMembers()
{
	delete metaData;
	metaData = NULL;
	if(projectBoundary)
		delete[] projectBoundary;
	projectBoundary = NULL;

	for(int i = 0; i < geo3DMaps.size(); i++)
		if(geo3DMaps[i]->getContainer() == this)
			delete geo3DMaps[i];

	geo3DMaps.clear();

	for(int i = 0; i < geoModels.size(); i++ )
		if(geoModels[i]->getContainer() == this)
			delete geoModels[i];

	geoModels.clear();
 
}
///@brief 获取GeoProject对应的元数据
///@param 
///@return 返回指向Geo3DProjectMetaData的指针
///@author zhaowei
///@date	 2013.11.1
Geo3DProjectMetaData* Geo3DProject::GetGeoProjectMetaData()
{
	return metaData;
}

///@brief  设置GeoProject对应的元数据
///@param value 待设置的Geo3DProjectMetaData指针
///@return 
///@author zhaowei
///@date   2013.11.08
void Geo3DProject::SetGeoProjectMetaData(Geo3DProjectMetaData* value)
{
	value->setContainer(this);
	if(metaData)
		delete metaData;
	metaData = value;
}

/// @name 场景中包含的地质图。
//@{
/// @brief  取场景中所包含地质图的数目。
/// @return 返回场景中地质图的数目。
/// @author zhaowei
/// @date   2013.11.07
int Geo3DProject::GetGeo3DMapCount()
{
	int map_count = geo3DMaps.size();
	return map_count;
}

/// @brief  取场景中的地质图。
/// @param  i 从0开始的地质图的索引号。
/// @return 返回场景中第i个地质图。若指定序号的地质图不存在则返回0。返回的指针所指向对象的内存由本对象管理。
/// @author zhaowei
/// @date   2013.11.07
Geo3DMap* Geo3DProject::GetGeo3DMap(const int& i)
{
	if(!(i >= 0 && i < geo3DMaps.size()))
		return NULL;		
	Geo3DMap* geo_map = geo3DMaps[i];		
	return geo_map;
}
//@}

///@brief  添加地质图
///@param Geo3DMap* geoMap 待添加的地质图
///@return 
///@author zhaowei
///@date   2013.11.06
void Geo3DProject::AddGeoMap(Geo3DMap* geoMap)
{
	if(geoMap)
	{
		geoMap->setContainer(this);
		geo3DMaps.push_back(geoMap);
	}
}

///@brief  获取所有地质图
///@param maps 存放地质图的容器
///        mapType 获取的地质图类型
///@return 
///@author zhaowei
///@date   2013.11.06
void Geo3DProject::GetAllGeoMaps(std::vector<Geo3DMap*> &maps, const std::string& mapType)
{
	int map_count = GetGeo3DMapCount();
	for(int i = 0; i < map_count; i++)
	{
		if(geo3DMaps[i]->GetType() == mapType)
		{
			maps.push_back(geo3DMaps[i]);
		}
	}
}

///@brief  设置工区边界坐标
///@param projectBoundary 工区边界的点坐标
///        count           工区边界的点数
///@return
///@author zhaowei
///@date   2013.11.06
void Geo3DProject::SetProjectBoundary(double* Boundary, const int& count)
{	
	// 如果ProjectBoundary在设置时不为空，那么需要先删除分配给它的内存空间，再对其赋值。
	// 以免造成先前分配给它的空间成为垃圾内存。
	if(projectBoundary)
		delete projectBoundary;
	projectBoundary = Boundary;
	pointCount = count;
}


///@brief  获取工区边界坐标
///@param 
///@return 表示工区边界点坐标的数组指针
///@author zhaowei
///@date   2013.11.06
double* Geo3DProject::GetProjectBoundary()
{
	return projectBoundary;
}

///@brief  获取工区边界点数
///@param  
///@return 工区边界点数
///@author zhaowei
///@date   2013.11.06
int Geo3DProject::GetProjectBoundaryPointCount()
{
	return pointCount;
}

///@brief  获取属性数目
///@param
///@return 返回属性数目
///@author zhaowei
///@date   2013.11.06
int Geo3DProject::GetAttributeValueCount()
{
	int attr_count = attributeValues.size();
	return attr_count;
}

///@brief  获取序号为i的属性
///@param i 序号
///@return 返回属性
///@author zhaowei
///@date   2013.11.06
AttributeValue Geo3DProject::GetAttributeValue(const int& i)
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
///@date   2013.11.06
AttributeValue Geo3DProject::GetAttributeValue(const std::string& fieldname)
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
///@date   2013.11.05
void Geo3DProject::AddAttributeValue(const AttributeValue& value)
{
	attributeValues.push_back(value);
}

///@brief  取场景中所包含地质模型的数目。
///@return 返回场景中地质模型的数目。
///@author zhaowei
///@date   2013.11.06
int Geo3DProject::GetGeoModelCount()
{
	int model_count = geoModels.size();
	return model_count;
}
///@brief  删除全部AttributeValue容器中添加新的元素
///@return 
///@author xiehaifeng
///@date   2013.12.26
void Geo3DProject::DeleteAllAttributeValue()
{
	attributeValues.clear();
	
}
///@brief  取场景中的地质模型。
///@param  i 从0开始的地质模型的索引号。
///@return 返回场景中第i个地质模型。若指定序号的地质模型不存在则返回0。返回的指针所指向对象的内存由本对象管理。
///@author zhaowei
///@date   2013.11.07
GeoModel* Geo3DProject::GetGeoModel(const int& i)
{
	if(!(i >= 0 && i < geoModels.size()))
		return NULL;		
	GeoModel* geo_model = geoModels[i];	
	return geo_model;
}

///@brief  添加地质模型
///@param  geoModel 待添加的地质模型
///@return 
///@author zhaowei
///@date   2013.11.06
void Geo3DProject::AddGeoModel(GeoModel* geoModel)
{
	if(geoModel)
	{
		geoModel->setContainer(this);
		geoModels.push_back(geoModel);
	}
}

///@brief  获取所有地质模型
///@param  models 存放地质模型的容器
///        modelType 获取的地质模型类型
///@return 
///@author zhaowei
///@date   2013.11.06
void Geo3DProject::GetAllGeoModels(std::vector<GeoModel*> &models, std::string modelType)
{
	int model_count = GetGeoModelCount();
	for(int i = 0; i < model_count; i++)
	{
		if(geoModels[i]->GetType() == modelType)
		{
			models.push_back(geoModels[i]);
		}
	}
}

void Geo3DProject::UpdateRefences()
{
	for(int i = 0; i < GetGeo3DMapCount(); i++)
	{
		Geo3DMap* geomap = GetGeo3DMap(i);
		for(int j = 0; j < geomap->GetGeo3DLayerCount(); j++)
		{
			Geo3DLayer* layer = geomap->GetLayer(j);
			if(layer->GetFeatureClass() == 0)
				continue;
			std::string fcid = layer->GetFeatureClass()->getID();
			int k = fcid.find("#");
			fcid = fcid.substr(k+1,fcid.length());
			for(int m = 0; m < geoModels.size(); m++)
			{
				GeologicFeatureClass* fc =  geoModels[m]->GetGeoFeatureClass(fcid);
				if(fc)
				{
					layer->SetFeatureClass(fc);
					break;
				}
			}
		}
	}

	for(int m = 0; m < geoModels.size(); m++)
	{
		geoModels[m]-> UpdateRelations();

	}
}
 