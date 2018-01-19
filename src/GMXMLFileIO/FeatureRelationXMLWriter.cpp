#include "FeatureRelationXMLWriter.h"
#include "GeoFeatureXMLWriter.h"
#include "GeologicFeature.h"
#include "FeatureRelation.h"
#include "FeatureClass.h"
#include "GeoModel.h"


FeatureRelationXMLWriter::FeatureRelationXMLWriter(void)
{
}


FeatureRelationXMLWriter::~FeatureRelationXMLWriter(void)
{
}

void FeatureRelationXMLWriter::WriteFeatureRelation(gmml::FeatureRelation* feature_relation, std::ofstream& xml_stream)
{
	xml_stream << "<Relation>" << std::endl;


	if(feature_relation)
	{
		const std::string re = feature_relation->GetClassName();
		xml_stream << "<" << re << " gml:id=\"" << feature_relation->getID()/* + "_Boundaries"*/ << "\">" << std::endl;
		xml_stream << "<gml:name>" << feature_relation->getName()/* + "_Boundaries"*/ << "</gml:name>" << std::endl;
		xml_stream << "<gml:Relationship>" << feature_relation->getRelationship()/* + "_Boundaries"*/ << "</gml:Relationship>" << std::endl;
		xml_stream << "<Source>" << std::endl;
		xml_stream << "<Feature xlink:type=\"simple\" xlink:href=\"" << feature_relation->getSourceRole() << "\" />" << std::endl;
		xml_stream << "</Source>" << std::endl;

		xml_stream << "<Targets>" <<   std::endl;
		int target_count = feature_relation->GetTargetCount();
		for (int j = 0; j < target_count; j++)
		{
			xml_stream << "<Feature xlink:type=\"simple\" xlink:href=\"#" << feature_relation->getTargetRole(j) << "\" />" << std::endl;
		}
		xml_stream << "</Targets>" << std::endl;
		xml_stream << "</"<<re<<">" << std::endl;
		xml_stream << "</Relation>" << std::endl;
	}
			

}

	
