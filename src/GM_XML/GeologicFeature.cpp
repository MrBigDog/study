#include "GeologicFeature.h"
#include "MappedFeature.h"

using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.13
GeologicFeature::GeologicFeature(void)
{
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.13
GeologicFeature::~GeologicFeature(void)
{
} 

///@brief  取要素的类型名。
///@param  
///@return 要素的类型名。
///@author zhaowei
///@date   2013.11.13
std::string GeologicFeature::GetFeatureType()
{
	return featureType;
}

///@brief  设置要素的类型名
///@param  value 待设置的要素类型
///@return 
///@author zhaowei
///@date   2013.11.13
void GeologicFeature::SetFeatureType(const std::string& value)
{
	featureType = value;
}

///@brief  获取GeologicFeature包含的MappedFeature数目
///@param  
///@return 返回MappedFeature数目
///@author zhaowei
///@date   2013.11.13
int GeologicFeature::GetMappedFeatureCount()
{
	int mapped_feature_count = mappedFeatures.size();
	return mapped_feature_count;
}

///@brief  获取指定序号的MappedFeature
///@param  i 序号
///@return 指向MappedFeature的指针
///@author zhaowei
///@date   2013.11.13
MappedFeature* GeologicFeature::GetMappedFeature(const int& i)
{
	if(!(i >= 0 && i < mappedFeatures.size()))
		return NULL;		
	MappedFeature* mapped_feature = mappedFeatures[i];		
	return mapped_feature;
}

///@brief  获取全部的MappedFeature
///@param  mapped_features 作为引用参数，用于保存全部的mappedFeature
///@return 
///@author zhaowei
///@date   2013.11.13
void GeologicFeature::GetAllMappedFeature(std::vector<MappedFeature*>& mapped_features)
{
	mapped_features = mappedFeatures;
}

///@brief  添加新的MappedFeature
///@param  value 待添加的MappedFeature
///@return 
///@author zhaowei
///@date   2013.11.08
void GeologicFeature::AddMappedFeature(MappedFeature* value)
{
	value->setContainer(this);
	mappedFeatures.push_back(value);
}

void  GeologicFeature::AddBoundaryFeature(GeologicFeature* f)
{
	boundaryFeatures.push_back(f);
}

int  GeologicFeature::GetBoundaryFeatureCount()
{
	return boundaryFeatures.size();
}

GeologicFeature* GeologicFeature::GetBoundaryFeature(int i)
{
	return boundaryFeatures[i];
}

///@brief  释放管理的内存
///@param  
///@return
///@author zhaowei
///@date   2013.11.13
void GeologicFeature::deleteMembers()
{
	for(std::vector<MappedFeature*>::size_type i = 0; i < mappedFeatures.size(); i++)
	{
		if(mappedFeatures[i]->getContainer() == this)
		{
			delete mappedFeatures[i];
		}
	}
	boundaryFeatures.clear();
	mappedFeatures.clear();
}