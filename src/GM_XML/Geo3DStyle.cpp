#include "Geo3DStyle.h"

using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.14
Geo3DStyle::Geo3DStyle(void)
{

}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.14
Geo3DStyle::~Geo3DStyle(void)
{
	deleteMembers();
}


void Geo3DStyle::deleteMembers()
{
	for(int i = 0; i < featureStyles_.size(); i++)
		delete featureStyles_[i];

	featureStyles_.clear();

		for(int i = 0; i < coverageStyles_.size(); i++)
		delete coverageStyles_[i];

	coverageStyles_.clear();
}

int Geo3DStyle::GetFeatureTypeStyleCount()
{
	return featureStyles_.size();
}
se::FeatureType3DStyle* Geo3DStyle::GetFeatureTypeStyle(int i)
{
	return featureStyles_[i];
}

// 
// 	///@brief  设置FeatureTypeStyle
// 	///@param  value 待设置的FeatureTypeStyle
// 	///@return
// 	///@author zhaowei
// 	///@date   2013.11.14
void Geo3DStyle::SetFeatureTypeStyle(se::FeatureType3DStyle* value)
{
	featureStyles_.push_back(value);
}


int Geo3DStyle::GetCoverageTypeStyleCount()
{
	return coverageStyles_.size();
}
se::CoverageType3DStyle* Geo3DStyle::GetCoverageTypeStyle(int i)
{
	return coverageStyles_[i];
}

// 
// 	///@brief  设置FeatureTypeStyle
// 	///@param  value 待设置的FeatureTypeStyle
// 	///@return
// 	///@author zhaowei
// 	///@date   2013.11.14
void Geo3DStyle::SetCoverageTypeStyle(se::CoverageType3DStyle* value)
{
	coverageStyles_.push_back(value);
}