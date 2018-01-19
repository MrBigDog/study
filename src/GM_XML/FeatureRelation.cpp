#include "FeatureRelation.h"

using namespace gmml;
std::string  BoundaryRelationshipTerm = "SpatialBound";
std::string  BoundaryRelationshipSourceRoleTerm = "Internal";
std::string  BoundaryRelationshipTargetRoleTerm = "Boundary";
std::string  SpatialTopologicalAggregationRelationshipTerm = "SpatialAggregation";
std::string  SpatialTopologicalAggregationRelationshipSourceRole = "Aggregation";
std::string  SpatialTopologicalAggregationRelationshipTargetRole = "Component";

///@brief  构造函数
///@author zhaowei
///@date   2013.11.13
FeatureRelation::FeatureRelation(void)
{
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.13
FeatureRelation::~FeatureRelation(void)
{
}

///@brief  设置两个Feature之间的关系
///@param  term 待设置的关系
///@return
///@author zhaowei
///@date   2013.11.13
void FeatureRelation::setRelationship(const std::string& term)
{
	relationshipTerm=term;
}

///@brief  获取关系
///@param
///@return 返回关系
///@author zhaowei
///@date   2013.11.13
std::string FeatureRelation::getRelationship() const
{
	return relationshipTerm;
}

///@brief  设置源角色
///@param  源角色字符串
///@return
///@author zhaowei
///@date   2013.11.13
void FeatureRelation::setSourceRole(const std::string& term)
{
	source = term;
}

///@brief  获取源角色
///@param  
///@return 返回源角色字符串
///@author zhaowei
///@date   2013.11.13
std::string FeatureRelation::getSourceRole() const
{
	return source;
}

int FeatureRelation::GetTargetCount()
{
	return targets.size();
}

///@brief  设置目标角色
///@param  目标角色字符串
///@return
///@author zhaowei
///@date   2013.11.13
void FeatureRelation::addTargetRole(const std::string& term)
{
	targets.push_back(term) ;
}


///@brief  获取目标角色
///@param  
///@return 返回目标角色字符串
///@author zhaowei
///@date   2013.11.13

std::string FeatureRelation::getTargetRole(int i) const
{
	return targets[i];
}

///@brief  清除内存管理的数据
///@param
///@return
///@author zhaowei
///@date   2013.11.13
void FeatureRelation::deleteMembers()
{
   targets.clear();
}
