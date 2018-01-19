#include "GeoFeatureXMLWriter.h"
#include "GeometryXMLWriter.h"
#include "GeologicFeature.h"
#include "MappedFeature.h"

#include "vtkPointData.h"
#include "../vtkExtending/GMPolyData.h"
#include "../vtkExtending/GMUnstructuredGrid.h"
#include "../vtkExtending/GMStructuredPoints.h"
#include "../vtkExtending/GMCornerPointGrid.h"
#include "vtkCellArray.h"
#include "vtkMath.h"
#include "vtkPoints.h"
#include "vtkPointData.h"
#include "vtkCellData.h"
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"

#include "vtkCharArray.h"
#include "vtkLongArray.h"
#include "vtkIntArray.h"
//#include "vtkPolyDataNormals.h"

GeoFeatureXMLWriter::GeoFeatureXMLWriter(void)
{
}


GeoFeatureXMLWriter::~GeoFeatureXMLWriter(void)
{
}

void GeoFeatureXMLWriter::WriteFeature(gmml::GeologicFeature* feature, std::ostream& xml_stream)
{
	std::string xml_node_name = "GeoFeature";
	xml_stream << "<" << xml_node_name << " gml:id=\"" << feature->getID() << "\">" << std::endl;
	xml_stream << "<gml:name>" << feature->getName() << "</gml:name>" << std::endl;

	xml_stream << "<Fields>" << std::endl;
	if(feature->GetAttributeValueCount() != 0)
	{
		for(int i = 0; i < feature->GetAttributeValueCount(); i++)
		{
			xml_stream << "<Field Name=\"" << feature->GetAttributeValue(i).fieldName << "\">"<< endl;

			xml_stream << "<swe:" << feature->GetAttributeValue(i).fieldType << ">" << endl;
			xml_stream << "<swe:value"  << ">" ;
			xml_stream << feature->GetAttributeValue(i).fieldValue;
			xml_stream << "</swe:value"  << ">" << endl;
			xml_stream << "</swe:"<< feature->GetAttributeValue(i).fieldType << ">" << endl;
			xml_stream << "</Field"  << ">" << endl;
		}

	}
	xml_stream << "</Fields>" << std::endl;
	for (int i = 0; i < feature->GetMappedFeatureCount(); i++)
	{
		gmml::MappedFeature* mapped_feature =  feature->GetMappedFeature(i);

		if (mapped_feature)
			WriteMappedFeature(mapped_feature, xml_stream);
	}
	xml_stream << "</" << xml_node_name << ">" << std::endl;
}

void GeoFeatureXMLWriter::WriteMappedFeature(gmml::MappedFeature* mapped_feature, std::ostream& xml_stream)
{	
	xml_stream << "<" << "Geometry>" << std::endl;
	if(mapped_feature->GetShape())
	{		
		xml_stream << "<Shape>" << std::endl;

		GeometryXMLWriter geometry_writer;
		geometry_writer.WriteGeometry(mapped_feature->GetShape(), xml_stream);
		xml_stream << "</Shape>" << std::endl;
	}		 

	//GeoMappedProperty
	WriteMappedProperty(mapped_feature, xml_stream);
	xml_stream << "</Geometry>" << std::endl;
}

void GeoFeatureXMLWriter::WriteMappedProperty(gmml::MappedFeature* mapped_feature, std::ostream& xml_stream)
{
	if(mapped_feature->GetShape())
	{
		vtkObject * ob = mapped_feature->GetShape();
		vtkDataSet* dataset = 0;
		std::string geoid = mapped_feature->getID() ;
		std::string strClassName = ob->GetClassName();
		if(strClassName == "GMPolyData")
		{
			GMPolyData* poly =( GMPolyData*)ob;
			dataset = poly;
			geoid = poly->GetID();
		}

		if(strClassName == "GMUnstructuredGrid")
		{
			GMUnstructuredGrid* poly =( GMUnstructuredGrid*)ob;
			dataset = poly;
			geoid = poly->GetID();
		}

		if(strClassName == "GMStructuredPoints")
		{
			GMStructuredPoints* poly =( GMStructuredPoints*)ob;
			dataset = poly;
			geoid = poly->GetID();
		}

		if(strClassName == "GMCornerPointGrid")
		{
			GMCornerPointGrid* poly =( GMCornerPointGrid*)ob;
			dataset = poly;
			geoid = poly->GetID();
		}
		

		if(strClassName == "GMCornerPointGrid")
		{
			xml_stream << "<Propertymodels>" << endl;

			GMCornerPointGrid* poly =( GMCornerPointGrid*)ob;
			 
			int num = poly->GetCountOfCellPropertyData();

			for(int n =0; n < num; n++)
			{
				WriteDiscreteCoverage(2, poly->GetCellPropertyData(n), xml_stream,geoid);
			}

			xml_stream << "<Description>" << "Propertymodel" ;
			xml_stream << "</Description>" << endl;
			xml_stream << "</Propertymodels>" << endl;

		}else
		if(dataset)
		{
			int num1 = dataset->GetPointData()->GetNumberOfArrays();
			int num2 = 0;
			if(dataset->GetCellData())
				num2 = dataset->GetCellData()->GetNumberOfArrays();

			if(num1+num2)
				xml_stream << "<Propertymodels>" << endl;
		 

			int num = dataset->GetPointData()->GetNumberOfArrays();
			if(num > 0)
			{
				for(int n =0; n < num; n++)
				{
					WriteDiscreteCoverage(1, dataset->GetPointData()->GetArray(n), xml_stream,geoid);
				}
			}
			if(dataset->GetCellData())
				num = dataset->GetCellData()->GetNumberOfArrays();

			if(num > 0)
			{
				for(int n = 0; n < num; n++)
				{
					WriteDiscreteCoverage(2,dataset->GetCellData()->GetArray(n),xml_stream,geoid);
				}
			}

			if(num1+num2)
			{
				xml_stream << "<Description>" << "Propertymodel" ;
				xml_stream << "</Description>" << endl;
				xml_stream << "</Propertymodels>" << endl;
			}
		}
	}
}

void GeoFeatureXMLWriter::WriteDiscreteCoverage(int position_type, vtkObject* shape_property, std::ostream& xml_stream, std::string geoid)
{
	vtkDataArray* data_array = (vtkDataArray *)shape_property;

	if(std::string(data_array->GetName()) == "Normals")
		return;
		
	xml_stream << "<GeoDiscreteCoverage>"<< std::endl;

	xml_stream << "<gml:domainSet xlink:type=\"simple\" xlink:href=\"#" <<  geoid << "\" />" << std::endl;

	xml_stream << "<GeoPropertyRange>"<< std::endl;

	if(position_type == 1)
	{
		xml_stream << "<PropertyPosType>" << "VERTEX" << "</PropertyPosType>" << std::endl;
	}
	else
	{
		if(position_type == 2)
			xml_stream << "<PropertyPosType>" << "FACE" << "</PropertyPosType>" << std::endl;
		else
			xml_stream << "<PropertyPosType>" << "VOLUME" << "</PropertyPosType>" << std::endl;
	}

	std::string data_type ;
	int data_count=0;
	int data_component=0;

	if(0 == strcmp(shape_property->GetClassName() , "vtkDoubleArray"))
	{
		vtkDoubleArray* dataA = (vtkDoubleArray*)shape_property;
		data_type = "double";
		data_count = dataA->GetNumberOfTuples();
		data_component = dataA->GetNumberOfComponents();
	}
	else if(0 == strcmp(shape_property->GetClassName(), "vtkFloatArray"))
	{
		vtkFloatArray* dataA = (vtkFloatArray*)shape_property;
		data_type = "float";
		data_count = dataA->GetNumberOfTuples();
		data_component = dataA->GetNumberOfComponents();
	}
	else if(0 == strcmp(shape_property->GetClassName() , "vtkLongArray"))
	{
		vtkLongArray* dataA = (vtkLongArray*)shape_property;
		data_type = "long";
		data_count = dataA->GetNumberOfTuples();
		data_component = dataA->GetNumberOfComponents();
	}
	else if(0 == strcmp(shape_property->GetClassName() , "vtkIntArray"))
	{
		vtkIntArray* dataA = (vtkIntArray*)shape_property;
		data_type = "int";
		data_count = dataA->GetNumberOfTuples();
		data_component = dataA->GetNumberOfComponents();
	}
	else if(0 == strcmp(shape_property->GetClassName() , "vtkCharArray"))
	{
		vtkCharArray* dataA = (vtkCharArray*)shape_property;
		data_type = "char";
		data_count = dataA->GetNumberOfTuples();
		data_component = dataA->GetNumberOfComponents();
	}
	else if(0 == strcmp(shape_property->GetClassName() , "vtkUnsignedCharArray"))
	{
		vtkUnsignedCharArray* dataA = (vtkUnsignedCharArray*)shape_property;
		data_type = "unsigned char";
		data_count = dataA->GetNumberOfTuples();
		data_component = dataA->GetNumberOfComponents();
	}

	vtkDataArray* vtkarray  = (vtkDataArray*)shape_property;
	xml_stream << "<GeoPropertyDataType>"<< std::endl;
	char buf[256] = {'\0'};
	sprintf(buf,"Row=\"%d\" Col=\"1\">",data_component);
	if(data_component == 1)
		xml_stream << "<PropertyFeildType Row=\"1\" Col=\"1\">" << "SCALARY" << "</PropertyFeildType>" << std::endl;
	else if(data_component==3)
			xml_stream << "<PropertyFeildType Row=\"3\" Col=\"1\">" << "VECTOR" << "</PropertyFeildType>" << std::endl;
	else if(data_component==9)
				xml_stream << "<PropertyFeildType Row=\"3\" Col=\"3\">" << "TENSOR" << "</PropertyFeildType>" << std::endl;
	else
		xml_stream << "<PropertyFeildType "<< buf << "MATRIX" << "</PropertyFeildType>" << std::endl;

	xml_stream << "</GeoPropertyDataType>" << std::endl;

	xml_stream << "<Values>" << std::endl;

	xml_stream << "<DataArray>" << std::endl;

	xml_stream << "<DataType>" <<data_type << std::endl;
	xml_stream << "</DataType>"<< std::endl;

	xml_stream << "<DataSize>" << data_count * data_component << std::endl;
	xml_stream << "</DataSize>" << std::endl;
	xml_stream << "<Data>" <<std::endl;

	for(int i = 0; i < data_count * data_component; i++)
	{
		if(data_type=="long")
		{
			vtkLongArray* dataA = (vtkLongArray*)shape_property;
			xml_stream << dataA->GetValue(i) << " ";
		}

		if(data_type=="int")
		{
			vtkIntArray* dataA = (vtkIntArray*)shape_property;
			xml_stream << dataA->GetValue(i) << " ";
		}
		if(data_type=="double")
		{
			vtkDoubleArray* dataA = (vtkDoubleArray*)shape_property;
			xml_stream << dataA->GetValue(i) << " ";
		}
		if(data_type=="float")
		{
			vtkFloatArray* dataA = (vtkFloatArray*)shape_property;
			xml_stream << dataA->GetValue(i) << " ";
		}
		if(data_type=="char")
		{
			vtkCharArray* dataA = (vtkCharArray*)shape_property;
			int d = dataA->GetValue(i);
			xml_stream << d <<" ";
		}
		if(data_type=="unsignedchar")
		{
			vtkUnsignedCharArray* dataA = (vtkUnsignedCharArray*)shape_property;
			int d = dataA->GetValue(i);
			xml_stream << d <<" ";
		}
		if(data_type=="unsigned char")
		{
			vtkUnsignedCharArray* dataA = (vtkUnsignedCharArray*)shape_property;
			int d = dataA->GetValue(i);
			xml_stream << d <<" ";
		}
	}

	xml_stream << std::endl; 
	xml_stream << "</Data>"<< std::endl;

	xml_stream << "</DataArray>"<< std::endl;

	xml_stream << "</Values>"<< std::endl;

	xml_stream << "</GeoPropertyRange>"<< std::endl;

	xml_stream << "<gmlcov:rangeType>"<< std::endl;

	xml_stream << "<swe:DataRecord>"<< std::endl;

	xml_stream << "<swe:field name=\"" << data_array->GetName() <<"\">"<< std::endl;

	xml_stream << "<swe:Quantity>" << std::endl;
	xml_stream <<"<swe:uom code=\"W/cm2\"/>" << std::endl;
	xml_stream << "</swe:Quantity>" << std::endl;

	xml_stream << "</swe:field>" << std::endl;

	xml_stream << "</swe:DataRecord>" << std::endl;

	xml_stream << "</gmlcov:rangeType>" << std::endl;

	xml_stream << "</GeoDiscreteCoverage>" << std::endl;
}
