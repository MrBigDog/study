#include "GeoModelXMLWriter.h"
#include "FeatureClassXMLWriter.h"
#include "FeatureRelationXMLWriter.h"
#include "FeatureClass.h"
#include "FeatureRelation.h"
#include "GeoModel.h"
#include "GeoModelMetaData.h"
#include "gmlAttribute.h"
GeoModelXMLWriter::GeoModelXMLWriter(void)
{
}


GeoModelXMLWriter::~GeoModelXMLWriter(void)
{
}

bool GeoModelXMLWriter::WriteModel(gmml::GeoModel* model, const std::string& xml_path,char*pEncoding)
{
	if(!model)
		return false;

	std::ofstream xml_file(xml_path.c_str());
	if (!xml_file.is_open())
		return false;
	char pDataBuf[100]={0};
	sprintf(pDataBuf,"<?xml version=\"1.0\" encoding=\"%s\"?>",pEncoding);
	xml_file << pDataBuf<< std::endl
	//xml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl
		<< "<" << "GeoModel" << " xmlns:geo3dml=\"" << xmlNS_GMML << "\" xmlns=\"" << xmlNS_GMML << "\"" << std::endl
		<< "\t" << "xmlns:gml=\"" << xmlNS_gml << "\"" << std::endl
		<< "\t" << "xmlns:dt=\"" << xmlNS_gml << "\"" << std::endl
		<< "\t" << "xmlns:gmlcov=\"" << xmlNS_gml << "\"" << std::endl
		<< "\t" << "xmlns:swe=\"" << xmlNS_swe << "\"" << std::endl
		<< "\t" << "xmlns:ogc=\"" << xmlNS_ogc << "\"" << std::endl
		<< "\t" << "xmlns:se=\"" << xmlNS_se << "\"" << std::endl
		<< "\t" << "xmlns:gsml=\"" << xmlNS_gsml << "\"" << std::endl
		<< "\t" << "xmlns:gsmlst=\"" << xmlNS_gsmlst << "\"" << std::endl
		<< "\t" << "xmlns:gsmlgu=\"" << xmlNS_gsmlgu << "\"" << std::endl
		<< "\t" << "xmlns:xlink=\"" << xmlNS_xlink << "\"" << std::endl
		<< "\t" << "xmlns:xsi=\"" << xmlNS_xsi << "\"" << std::endl
		<< "\t" << "xmlns:xi=\"http://www.w3.org/2001/XInclude\"" << std::endl
		<< "\t" << "xmlns:GMML=\"http://www.cgs.gov.cn/GeoGML\"" << std::endl
		<< "\t" << "xsi:schemaLocation=\"" << xmlNS_GMML << " file:/E:/GMML/docs/schema/gmml.xsd\">" << std::endl;

	xml_file << "<Name>" << model->getName();
	xml_file << "</Name>" <<std::endl;

	xml_file << "<Type>" << model->GetType();
	xml_file << "</Type>" <<std::endl;

	xml_file << "<Metadata>" <<std::endl;

	int attrs =0;

	if(model)
	if(model->GetGeoModelMetaData())
	    attrs = model->GetGeoModelMetaData()->GetAttributeValueCount();


		for(int i = 0; i < attrs; i++)
		{

			AttributeValue attr =  model->GetGeoModelMetaData()->GetAttributeValue(i);
			xml_file << "<" <<attr.fieldName<<">";
			xml_file <<attr.fieldValue;
			xml_file << "</" <<attr.fieldName<<">" <<std::endl;

		}
	
		

	xml_file << "</Metadata>" <<std::endl;

	int fc_count = model->GetGeoFeatureClassCount();
	xml_file << "<FeatureClasses>" <<std::endl;

	for(int i = 0; i < fc_count; i++)
	{
		FeatureClassXMLWriter fc_writer;
		fc_writer.WriteFeatureClass(model->GetGeoFeatureClass(i), xml_file);
	}

	xml_file << "</FeatureClasses>" <<std::endl;

	xml_file << "<FeatureRelationship>" <<std::endl;
	int fr_count = model->GetGeoFeatureRelationCount();
	for(int i = 0; i < fr_count; i++)
	{
		FeatureRelationXMLWriter fr_writer;
		gmml::FeatureRelation* feature_relation = model->GetGeoFeatureRelation(i);
		fr_writer.WriteFeatureRelation(feature_relation, xml_file);
	}
	xml_file << "</FeatureRelationship>" <<std::endl;
	xml_file << "</GeoModel>" << std::endl;

	return true;
}
