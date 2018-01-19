#ifndef GeologicFeature_h__
#define GeologicFeature_h__

#include "AbstractFeature.h" 

namespace gmml
{
	class MappedFeature;
	class GEOXML_EXPORT GeologicFeature : public gml::AbstractFeature
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.13
		GeologicFeature(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.13
		~GeologicFeature(void);

		///@brief  获取该类的名称
		///@param 
		///@return 返回类名字
		///@author zhaowei
		///@date   2013.11.13
		std::string GetClassName() { return "GeologicFeature"; };

		///@brief  取要素的类型名。
		///@param  
		///@return 要素的类型名。
		///@author zhaowei
		///@date   2013.11.13
		std::string GetFeatureType();

		///@brief  设置要素的类型名
		///@param  value 待设置的要素类型
		///@return 
		///@author zhaowei
		///@date   2013.11.13
		void SetFeatureType(const std::string& value);

		///@brief  获取GeologicFeature包含的MappedFeature数目
		///@param  
		///@return 返回MappedFeature数目
		///@author zhaowei
		///@date   2013.11.13
		int GetMappedFeatureCount();

		///@brief  获取指定序号的MappedFeature
		///@param  i 序号
		///@return 指向MappedFeature的指针
		///@author zhaowei
		///@date   2013.11.13
		MappedFeature* GetMappedFeature(const int& i);

		///@brief  获取全部的MappedFeature
		///@param  mapped_features 作为引用参数，用于保存全部的mappedFeature
		///@return 
		///@author zhaowei
		///@date   2013.11.13
		void GetAllMappedFeature(std::vector<MappedFeature*>& mapped_features);

		///@brief  添加新的MappedFeature
		///@param  value 待添加的MappedFeature
		///@return 
		///@author zhaowei
		///@date   2013.11.08
		void AddMappedFeature(MappedFeature* value);

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();

		///@brief 添加与本地质要素有边界关系的其他要素
		///@param f 有边界关联的其他地质要素
		///@author wang
		///@date 2013.11.26
		void AddBoundaryFeature(GeologicFeature* f);

		///@brief 获取有边界关系的要素数组的数目
		///@return 返回有边界关系的要素数目
		///@author wang
		///@date 2013.11.26
		int  GetBoundaryFeatureCount();

		///@brief 获取指定序号的有边界联系的其他要素
		///@param i 序号
		///@return 返回指定序号的地质要素的指针
		///@author wang
		///@date 2013.11.26
		GeologicFeature* GetBoundaryFeature(int i);
	protected:
		/// 要素类型
		std::string featureType;
		/// 要素对应的几何体数组
		std::vector<MappedFeature*>  mappedFeatures; // Geometry
		/// 与本要素有边界关系的其他要素数组
		std::vector<GeologicFeature*>  boundaryFeatures;
	};

}

#endif // GeologicFeature_h__
