#include "GeoModel.h"
#include "GeoModelMetaData.h"
#include "FeatureRelation.h"
#include "GeologicFeature.h"
#include "FeatureClass.h"
using namespace gmml;

///@brief  构造函数
///@author zhaowei
///@date   2013.11.08
GeoModel::GeoModel(void)
{
	metaData = new GeoModelMetaData;
	metaData->setContainer(this);
}

///@brief  析构函数
///@author zhaowei
///@date   2013.11.08
GeoModel::~GeoModel(void)
{
	deleteMembers();

}

///@brief  获取GeoModel对应的元数据
///@param
///@return 返回指向GeoModelMetaData的指针
///@author zhaowei
///@date   2013.11.08
GeoModelMetaData* GeoModel::GetGeoModelMetaData()
{
	return metaData;
}

///@brief  设置GeoModel对应的元数据
///@param value 待设置的元数据
///@return 
///@author zhaowei
///@date   2013.11.08
void GeoModel::SetGeoModelMetaData(GeoModelMetaData* value)
{
	value->setContainer(this);
	metaData = value;
}

///@brief  获取模型类型
///@param
///@return 返回该模型的类型
///@author zhaowei
///@date   2013.11.08
std::string GeoModel::GetType()
{
	return geoModelType;
}

///@brief  设置模型类型
///@param value 待设置的模型类型
///@return
///@author zhaowei
///@date   2013.11.08
void GeoModel::SetType(const std::string& value)
{
	geoModelType = value;
}

///@brief  获取属性数目
///@param
///@return 返回属性数目
///@author zhaowei
///@date   2013.11.08
int GeoModel::GetAttributeValueCount()
{
	int attr_count = attributeValues.size();
	return attr_count;
}

///@brief  获取序号为i的属性
///@param i 序号
///@return 返回属性
///@author zhaowei
///@date   2013.11.08
AttributeValue GeoModel::GetAttributeValue(const int& i)
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
///@param fieldname 属性名称
///@return 返回属性
///@author zhaowei
///@date   2013.11.08
AttributeValue GeoModel::GetAttributeValue(const std::string& fieldname)
{
	if(fieldname.empty())
	{
		AttributeValue v;
		v.fieldName = "";
		v.fieldValue = "";
		return v;
	}
	for(int i = 0; i < attributeValues.size(); i++)
	{
		if(attributeValues[i].fieldName == fieldname)
		{
			return attributeValues[i];
		}
	}
}

///@brief  向AttributeValue容器中添加新的元素
///@param  const AttributeValue& value 待添加的新元素
///@return 
///@author zhaowei
///@date   2013.11.08
void GeoModel::AddAttributeValue(const AttributeValue& value)
{
	attributeValues.push_back(value);
}

///@brief  获取指定序号的FeatureClass
///@param  i 序号
///@return 返回指向FeatureClass的指针
///@author zhaowei
///@date   2013.11.08
GeologicFeatureClass* GeoModel::GetGeoFeatureClass(const int& i)
{
	if(i < 0 || i >= geoFeatureClasses.size())
	{
		return NULL;
	}
	return geoFeatureClasses[i];
}

///@brief 获取指定ID的FeatureClass
///@param fcID FeatureClass的ID
///@return 返回FeatureClass指针
///@author wang
///@date 2013.11.26
GeologicFeatureClass* GeoModel::GetGeoFeatureClass(const std::string& fcID)
{
	for(int i = 0;  i < geoFeatureClasses.size(); i++)
	{
		if(geoFeatureClasses[i]->getID() == fcID)
			return geoFeatureClasses[i];
	}
	return NULL;
}

///@brief  获取全部的FeatureClass
///@param  geo_feature_class 作为引用参数，保存取出的FeatureClass
///@return 
///@author zhaowei
///@date   2013.11.08
void GeoModel::GetAllFeatureClass(std::vector<GeologicFeatureClass*>& geo_feature_class)
{
	geoFeatureClasses = geo_feature_class;
}

///@brief  添加新的FeatureClass
///@param  value 待添加的FeatureClass
///@return 
///@author zhaowei
///@date   2013.11.08
void GeoModel::AddFeatureClass(GeologicFeatureClass* value)
{
	geoFeatureClasses.push_back(value);
	value->setContainer(this);	
}


///@brief  获取指定序号的FeatureRelation
///@param i 序号
///@return 返回指向FeatureRelation的指针
///@author zhaowei
///@date   2013.11.08
FeatureRelation* GeoModel::GetGeoFeatureRelation(const int& i)
{
	if(i < 0 || i >=  featureRelations.size())
		return NULL;
	return featureRelations[i];
}

///@brief  获取全部的FeatureRelation
///@param  geo_feature_relation 作为引用参数，保存取出的FeatureRelation
///@return 
///@author zhaowei
///@date   2013.11.08
void GeoModel::GetAllFeatureRelation(std::vector<FeatureRelation*>& geo_feature_relation)
{
	featureRelations = geo_feature_relation;
}

///@brief  添加新的FeatureRelation
///@param  value 待添加的FeatureRelation
///@return 
///@author zhaowei
///@date   2013.11.08
void GeoModel::AddFeatureRelation(FeatureRelation* value)
{
	if (value)
	{
		featureRelations.push_back(value);
		value->setContainer(this);
	}
}

///@brief  释放管理的内存
///@param  
///@return
///@author zhaowei
///@date   2013.11.13
void GeoModel::deleteMembers()
{
	if(metaData)
		delete metaData;
	    metaData  = NULL;

	for(std::vector<GeologicFeatureClass*>::size_type i = 0; i < geoFeatureClasses.size(); i++)
	{
		if(geoFeatureClasses[i]->getContainer() == this);
		{
			delete geoFeatureClasses[i];
		}
	}
	geoFeatureClasses.clear();

	for(std::vector<FeatureRelation*>::size_type i = 0; i < featureRelations.size(); i++)
	{
		if(featureRelations[i]->getContainer() == this);
		{
			delete featureRelations[i];
		}
	}
	featureRelations.clear();
}

int GeoModel::GetGeoFeatureClassCount()
{
	return	geoFeatureClasses.size();
}

int GeoModel::GetGeoFeatureRelationCount()
{
	return	featureRelations.size();
}

///@brief 获取指定Model下ID确定的GeoFeature
///@param model 指定的GeoModel指针
///@param fid 确定的GeoFeatureID
///@return 返回GeoFeature的指针
///@author wang
///@date 2013.11.26
GeologicFeature* GetFeature(GeoModel* model, std::string fid)
{
	for(int j = 0; j < model->GetGeoFeatureClassCount() ; j++)
	{
		GeologicFeatureClass* fc =  model->GetGeoFeatureClass(j);
		for(int i = 0; i < fc->GetGeologicFeatureCount(); i++)
		{
			GeologicFeature *f =  fc->GetGeologicFeature(i);

			if(f->getID() == fid)
			{
				return f;
			}
		}
	}
	return 0;
}

void GeoModel::UpdateRelations()
{
	for(int i = 0; i < featureRelations.size(); i++)
	{
		if( featureRelations[i]->GetClassName() != "BoundaryRelationship")
			continue;

		std::string  sid = featureRelations[i]->getSourceRole();
		int b = sid.find("#");
		sid = sid.substr(b+1,sid.length());

		GeologicFeature* sourceFeature = GetFeature(this,sid);

		if(sourceFeature)
			for(int j = 0; j < featureRelations[i]->GetTargetCount(); j++)
			{
				std::string  tid = featureRelations[i]->getTargetRole(j) ;

				int bc = tid.find("#");
				tid = tid.substr(bc+1,tid.length());

				GeologicFeature * targetFeature = GetFeature(this,tid);
				if(targetFeature)
				{
					sourceFeature->AddBoundaryFeature(targetFeature);
				}
			}
	}
}