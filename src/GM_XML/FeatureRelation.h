#ifndef FeatureRelation_h__
#define FeatureRelation_h__

#include "AbstractGML.h"

namespace gmml
{
	static std::string BoundaryRelationshipTerm;
	static std::string BoundaryRelationshipSourceRoleTerm;
	static std::string BoundaryRelationshipTargetRoleTerm;
	static std::string SpatialTopologicalAggregationRelationshipTerm;
	static std::string SpatialTopologicalAggregationRelationshipSourceRole;
	static std::string SpatialTopologicalAggregationRelationshipTargetRole;

	class GEOXML_EXPORT FeatureRelation : public gml::AbstractGML
	{
	public:
		///@brief  构造函数
		///@author zhaowei
		///@date   2013.11.13
		FeatureRelation(void);

		///@brief  析构函数
		///@author zhaowei
		///@date   2013.11.13
		~FeatureRelation(void);

		///@brief  获取该类的名称
		///@param
		///@return 返回类的名字
		///@author zhaowei
		///@date   2013.11.13
		virtual std::string GetClassName() { return "FeatureRelation"; };

		///@brief  设置两个Feature之间的关系
		///@param  term 待设置的关系
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void setRelationship(const std::string& term);

		///@brief  获取关系
		///@param
		///@return 返回关系
		///@author zhaowei
		///@date   2013.11.13
		std::string getRelationship() const;

		///@brief  设置源角色
		///@param  源角色字符串
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void setSourceRole(const std::string& term);

		///@brief  获取源角色
		///@param  
		///@return 返回源角色字符串
		///@author zhaowei
		///@date   2013.11.13
		std::string getSourceRole() const;

		///@brief  设置目标角色
		///@param  目标角色字符串
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void addTargetRole(const std::string& term);

		///@brief  获取目标角色
		///@param  
		///@return 返回目标角色字符串
		///@author zhaowei
		///@date   2013.11.13	
		std::string getTargetRole(int) const;

		///@brief 获取目标角色数目
		///@return 返回目标角色数目
		///@author wang
		///@date 2013.11.26
		int GetTargetCount();
		//@}

		///@brief  清除内存管理的数据
		///@param
		///@return
		///@author zhaowei
		///@date   2013.11.13
		void deleteMembers();
	private:
		/// 要素之间的关系描述
		std::string relationshipTerm;
		/// 源要素
		std::string source;
		/// 目标要素数组
		std::vector<std::string> targets;

	};

}

#endif // FeatureRelation_h__
