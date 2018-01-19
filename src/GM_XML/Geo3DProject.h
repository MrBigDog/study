#ifndef Geo3DProject_h__
#define Geo3DProject_h__

#include "AbstractGML.h"

namespace gmml
{
	class Geo3DProjectMetaData;
	class Geo3DMap;
	class GeoModel;
	class GeoSceneStyle;
	/// 地质工程
	class GEOXML_EXPORT Geo3DProject : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.06
		Geo3DProject(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.06
		~Geo3DProject(void);

		///@brief  获取该类的名称
		///@param
		///@return 返回该类的名称
		///@author zhaowei
		///@date   2013.11.06
		std::string GetClassName() { return "Geo3DProject"; };

	public:
		/// @name   场景中包含的地质图。
		//@{
		/// @brief  取场景中所包含地质图的数目。
		/// @return 返回场景中地质图的数目。
		/// @author zhaowei
		/// @date   2013.11.07
		int GetGeo3DMapCount();
		/// @brief  取场景中的地质图。
		/// @param  i 从0开始的地质图的索引号。
		/// @return 返回场景中第i个地质图。若指定序号的地质图不存在则返回0。返回的指针所指向对象的内存由本对象管理。
		/// @author zhaowei
		/// @date   2013.11.07
		Geo3DMap* GetGeo3DMap(const int& i);
		//@}

		  ///@brief  添加地质图
		  ///@param  Geo3DMap* geoMap 待添加的地质图
		  ///@return 
		  ///@author zhaowei
		  ///@date   2013.11.06
		void AddGeoMap(Geo3DMap* geoMap);

		///@brief  获取所有地质图
		///@param  maps 存放地质图的容器
		///        mapType 获取的地质图类型
		///@return 
		///@author zhaowei
		///@date   2013.11.06
		void GetAllGeoMaps(std::vector<Geo3DMap*> &maps, const std::string& mapType);

		///@brief  设置工区边界坐标
		///@param  projectBoundary 工区边界的点坐标
		///        count           工区边界的点数
		///@return
		///@author zhaowei
		///@date   2013.11.06
		void SetProjectBoundary(double* projectBoundary, const int& count);

		///@brief  获取工区边界坐标
		///@param  
		///@return 表示工区边界点坐标的数组指针
		///@author zhaowei
		///@date   2013.11.06
		double* GetProjectBoundary();

		///@brief  获取工区边界点数
		///@param  
		///@return 工区边界点数
		///@author zhaowei
		///@date   2013.11.06
		int GetProjectBoundaryPointCount();

		///@brief 获取GeoProject对应的元数据
		///@param 
		///@return 返回指向Geo3DProjectMetaData的指针
		///@author zhaowei
		///@date	 2013.11.01
		Geo3DProjectMetaData* GetGeoProjectMetaData();

		///@brief  设置GeoProject对应的元数据
		///@param  value 待设置的Geo3DProjectMetaData指针
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void SetGeoProjectMetaData(Geo3DProjectMetaData* value);

		///@brief  获取属性数目
		///@param
		///@return 返回属性数目
		///@author zhaowei
		///@date   2013.11.06
		int GetAttributeValueCount();

		///@brief  获取序号为i的属性
		///@param  i 序号
		///@return 返回属性
		///@author zhaowei
		///@date   2013.11.06
		AttributeValue GetAttributeValue(const int& i);

		///@brief  获取属性名为fieldname的属性
		///@param  fieldname 属性名称
		///@return 如果属性名称为空，返回属性名称和属性值为空的对象；否则返回相应的属性
		///@author zhaowei
		///@date   2013.11.06
		AttributeValue GetAttributeValue(const std::string& fieldname);

		///@brief  向AttributeValue容器中添加新的元素
		///@param  const AttributeValue& value 待添加的新元素
		///@return 
		///@author zhaowei
		///@date   2013.11.05
		void AddAttributeValue(const AttributeValue& value);

		///@brief  取场景中所包含地质模型的数目。
		///@return 返回场景中地质模型的数目。
		///@author zhaowei
		///@date   2013.11.06
		int GetGeoModelCount();

		///@brief  取场景中的地质模型。
		///@param  i 从0开始的地质模型的索引号。
		///@return 返回场景中第i个地质模型。若指定序号的地质模型不存在则返回0。返回的指针所指向对象的内存由本对象管理。
		///@author zhaowei
		///@date   2013.11.07
		GeoModel* GetGeoModel(const int& i);

		///@brief  添加地质模型
		///@param  geoModel 待添加的地质模型
		///@return 
		///@author zhaowei
		///@date   2013.11.06
		void AddGeoModel(GeoModel* geoModel);

		///@brief  获取所有地质模型
		///@param  models 存放地质模型的容器
		///        modelType 获取的地质模型类型
		///@return 
		///@author zhaowei
		///@date   2013.11.06
		void GetAllGeoModels(std::vector<GeoModel*> &models, std::string modelType);

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();

		///@brief 建立Layer与对应FeatureClass的关联，并添加Model中的所有Feature的FeatureRelation
		///@author wang
		///@date 2013.11.26
		void UpdateRefences();
		///@brief  删除全部AttributeValue容器中添加新的元素
		///@return 
		///@author xiehaifeng
		///@date   2013.12.26
		void DeleteAllAttributeValue();

		void setDescription(std::string);
		std::string getDescription();

		GeoSceneStyle*           GetSceneStyle();
	protected:
		std::vector<AttributeValue>  attributeValues;
		Geo3DProjectMetaData*    metaData;
		std::vector<Geo3DMap*>   geo3DMaps;
		std::vector<GeoModel*>   geoModels;
		double*                  projectBoundary;
		int                      pointCount;
		GeoSceneStyle*           sceneStyle;
		std::string              projDescription;

	};

}
#endif // Geo3DProject_h__