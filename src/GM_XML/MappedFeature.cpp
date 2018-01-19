#include "MappedFeature.h"
using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.13
MappedFeature::MappedFeature(void)
{
	shape = NULL;
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.13
MappedFeature::~MappedFeature(void)
{
}

///@brief  释放管理的内存
///@param  
///@return
///@author zhaowei
///@date   2013.11.13
void MappedFeature::deleteMembers()
{
	if(shape)
		delete shape;
	shape = NULL;
}

///@brief  获取几何数据
///@param
///@return 指向几何数据的指针
///@author zhaowei
///@date   2013.11.13
vtkObject* MappedFeature::GetShape()
{
	return shape;
}

///@brief  设置几何数据
///@param  value 待设置的几何数据
///@return 
///@author zhaowei
///@date   2013.11.13
void MappedFeature::SetShape(vtkObject* value)
{
	shape = value;
}

bool MappedFeature::isShaped()
{
	return shape;
}

