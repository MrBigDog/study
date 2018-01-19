#ifndef MappedFeature_h__
#define MappedFeature_h__

#include "AbstractFeature.h" 

class vtkObject;

namespace gmml
{
	// MappedFeature == Geo3DML::Geometry
	class GEOXML_EXPORT MappedFeature : public gml::AbstractFeature
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.13
		MappedFeature(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.13
		~MappedFeature(void);

		///@brief  获取该类的名称
		///@param
		///@return 返回类的名字
		///@author zhaowei
		///@date   2013.11.13
		virtual std::string GetClassName() { return "MappedFeature"; };

		///@brief  释放管理的内存
		///@param  
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();

		///@brief  获取几何数据
		///@param
		///@return 指向几何数据的指针
		///@author zhaowei
		///@date   2013.11.13
		vtkObject* GetShape();

		///@brief  设置几何数据
		///@param  value 待设置的几何数据
		///@return 
		///@author zhaowei
		///@date   2013.11.13
		void SetShape(vtkObject* value);

		///@brief 判断几何数据是否存在
		///@author zhaowei
		///@date 2014.03.20
		bool isShaped();


	protected:
		vtkObject* shape;
	};

}

#endif // MappedFeature_h__
