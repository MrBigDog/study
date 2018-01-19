//#include "StdAfx.h"
#include "ShapePropertyReader.h"

#include "vtkObject.h"
#include "vtkLongArray.h"
#include "vtkDoubleArray.h"
#include "GMPolyData.h"
#include "GMStructuredPoints.h"
#include "GMUnstructuredGrid.h"
#include "vtkCellData.h"
#include "vtkPointData.h"
#include "vtkCharArray.h"
#include "vtkFloatArray.h"
#include "GMCornerPointGrid.h"

#include <sstream>

using std::istringstream;

vtkObject* pVtkObj = NULL;

int ReadShapeProperty(void* pData, xmlTextReaderPtr reader)
{
	xmlChar* name = NULL, *pre_name = NULL;
	int nRet = 1;


	pVtkObj = (vtkObject*)pData;

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name, BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
		}
		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(pre_name, BAD_CAST"Description"))
			{
				if (!xmlStrcmp(name, BAD_CAST"Propertymodels"))
				{

				}
			}
			else if (!xmlStrcmp(name, BAD_CAST"GeoDiscreteCoverage"))
			{
				if (!xmlStrcmp(pre_name, BAD_CAST"ShapeProperty") ||
					!xmlStrcmp(pre_name, BAD_CAST"GeometryProperty"))
				{
					std::vector<vtkDataArray*> dataarrays;
					std::vector<int> dataarraytypes;
					std::vector<std::string> dataarraysname;

					ReadDiscreteCoverage(reader, dataarrays, dataarraytypes, dataarraysname);
				}

				else if (!xmlStrcmp(pre_name, BAD_CAST"PropertyModels") ||
					!xmlStrcmp(pre_name, BAD_CAST"Propertymodels"))
				{
					ReadDiscreteCoverageGMML(0, reader);
				}

			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"Propertymodels") ||
				!xmlStrcmp(name, BAD_CAST"ShapeProperty") ||
				!xmlStrcmp(name, BAD_CAST"GeometryProperty") ||
				!xmlStrcmp(name, BAD_CAST"PropertyModels"))
			{
				return 0;
			}
		}

		if (pre_name)
		{
			xmlFree(pre_name);
		}

		if (name)
		{
			pre_name = name;
		}

		nRet = xmlTextReaderRead(reader);
	}

	return 0;
}

int ReadDiscreteCoverage(xmlTextReaderPtr reader, std::vector<vtkDataArray*> &dataarrays, std::vector<int> &dataarraytypes, std::vector<std::string>& dataarraysname)
{
	xmlChar* name = NULL;
	int nRet = 1;

	std::vector<vtkDataArray*> mydataarrays;
	std::vector<std::string> mydataarraysname;
	std::vector<int> mydataarraystype;

	char* out = 0;

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name, BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"ValueArray"))
			{
				mydataarrays.push_back(ReadValueArray(reader));
			}
			else if (!xmlStrcmp(name, BAD_CAST"swe:field"))
			{
				out = ReadAttrAndConvert(reader, "name");
			}
			else if (!xmlStrcmp(name, BAD_CAST"swe:Quantity"))
			{
				if (out)
				{
					mydataarraysname.push_back(out);
					mydataarraystype.push_back(1);

					out = NULL;
				}

			}
			else if (!xmlStrcmp(name, BAD_CAST"Vector"))
			{
				if (out)
				{
					mydataarraysname.push_back(out);
					mydataarraystype.push_back(2);

					out = NULL;
				}

			}
			else if (!xmlStrcmp(name, BAD_CAST"DataArray"))
			{
				if (out)
				{
					mydataarraysname.push_back(out);
					mydataarraystype.push_back(3);

					out = NULL;
				}

			}
		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"GeoDiscreteCoverage"))
			{
				if (mydataarrays.size() == mydataarraysname.size())
				{
					if (pVtkObj)
						for (int i = 0; i < mydataarrays.size() && i < mydataarraysname.size(); i++)
						{
							mydataarrays[i]->SetName(mydataarraysname[i].c_str());
						}

					if (pVtkObj)
						for (int i = 0; i < mydataarrays.size(); i++)
							dataarrays.push_back(mydataarrays[i]);
				}


				return 0;
			}
		}
		nRet = xmlTextReaderRead(reader);
	}

	return 0;
}

vtkDataArray* ReadValueArray(xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;

	vtkDataArray* data = 0;
	std::vector<double> dataarray;
	int dataType = 0; // 0:double   1:long  

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name, BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"swe:Quantity"))
			{
				dataType = 0;

				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					dataarray.push_back(atof((const char *)value));
					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name, BAD_CAST"Count"))
			{
				dataType = 1;

				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					dataarray.push_back(atoi((const char *)value));
					xmlFree(value);
				}
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"ValueArray"))
			{
				if (dataType == 1)
				{
					vtkLongArray * vtkdataarray = vtkLongArray::New();
					data = vtkdataarray;

					long *datavalues = new long[dataarray.size()];

					for (int i = 0; i < dataarray.size(); i++)
						datavalues[i] = dataarray[i];
					vtkdataarray->SetArray(datavalues, dataarray.size(), 0);
				}

				if (dataType == 0)
				{
					vtkDoubleArray * vtkdataarray = vtkDoubleArray::New();
					data = vtkdataarray;

					double *datavalues = new double[dataarray.size()];

					for (int i = 0; i < dataarray.size(); i++)
						datavalues[i] = dataarray[i];
					vtkdataarray->SetArray(datavalues, dataarray.size(), 0);
				}

				return data;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}
}

vtkDataArray* ReadDataArray(xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;

	xmlChar *szKey;          //ÁÙÊ±×Ö·û´®±äÁ¿
	GMPolyData* point = 0;

	vtkDataArray* data = 0;
	std::string datatype;
	int datacount = 0;
	int datacompont;

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name, BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
			continue;
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"DataType"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					int leftc = TrimLeft((char*)value);
					int rightc = TrimRight((char*)value);
					int len = strlen((char*)value);
					datatype = (char*)value;
					datatype = datatype.substr(leftc, len - rightc - leftc);
					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name, BAD_CAST"DataSize"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					datacount = atoi((const char *)value);
					xmlFree(value);
				}
			}
			else if (!xmlStrcmp(name, BAD_CAST"Data"))
			{
				xmlChar* value = xmlTextReaderReadString(reader);
				if (value)
				{
					if (datatype == "double")
					{
						vtkDoubleArray *doubledata = vtkDoubleArray::New();
						double* datapointer = doubledata->WritePointer(0, datacount);

						data = doubledata;
						char* buf = (char*)value;
						int offset = 0;

						istringstream istr(buf);;
						for (int i = 0; i < datacount; i++)
						{
							double a;
							istr >> a;
							datapointer[i] = a;
						}

					}

					if (datatype == "float")
					{
						vtkFloatArray *doubledata = vtkFloatArray::New();
						float* datapointer = doubledata->WritePointer(0, datacount);
						data = doubledata;
						istringstream istr((char*)value);;
						for (int i = 0; i < datacount; i++)
						{
							float a;
							istr >> a;
							datapointer[i] = a;
						}
					}

					if (datatype == "char")
					{
						vtkCharArray *doubledata = vtkCharArray::New();
						char*  datapointer = doubledata->WritePointer(0, datacount);
						data = doubledata;
						char* buf = (char*)value;

						istringstream istr((char*)value);;
						for (int i = 0; i < datacount; i++)
						{
							short a;
							istr >> a;
							datapointer[i] = a;
						}
					}
					if (datatype == "unsignedchar")
					{
						vtkUnsignedCharArray *doubledata = vtkUnsignedCharArray::New();
						unsigned char*  datapointer = doubledata->WritePointer(0, datacount);
						data = doubledata;
						char* buf = (char*)value;

						istringstream istr((char*)value);;
						for (int i = 0; i < datacount; i++)
						{
							short a;
							istr >> a;
							datapointer[i] = a;
						}
					}
					if (datatype == "unsigned char")
					{
						vtkUnsignedCharArray *doubledata = vtkUnsignedCharArray::New();
						unsigned char*  datapointer = doubledata->WritePointer(0, datacount);
						data = doubledata;
						char* buf = (char*)value;

						istringstream istr((char*)value);;
						for (int i = 0; i < datacount; i++)
						{
							short a;
							istr >> a;
							datapointer[i] = a;
						}
					}
					if (datatype == "long" || datatype == "short" || datatype == "int")
					{
						vtkLongArray *doubledata = vtkLongArray::New();
						long*  datapointer = doubledata->WritePointer(0, datacount);
						data = doubledata;
						char* buf = (char*)value;
						istringstream istr((char*)value);;
						for (int i = 0; i < datacount; i++)
						{
							long a;
							istr >> a;
							datapointer[i] = a;
						}
					}

					xmlFree(value);
				}
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"Values"))
			{
				return data;
			}
		}

		nRet = xmlTextReaderRead(reader);
	}
}

int ReadDiscreteCoverageGMML(void* pData, xmlTextReaderPtr reader)
{
	xmlChar* name = NULL;
	int nRet = 1;

	vtkDataArray* data = 0;
	int Row = 0, Col = 0, compount = 0;

	std::string PropertyPosType;
	std::string PropertyFeildType;
	std::string PropertyFeildName;

	while (nRet == 1)
	{
		name = xmlTextReaderName(reader);
		if (!xmlStrcmp(name, BAD_CAST"#text"))
		{
			nRet = xmlTextReaderRead(reader);
		}

		if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"swe:field"))
			{
				char* out = ReadAttrAndConvert(reader, "name");
				if (out)
				{
					PropertyFeildName = out;
					data->SetName(PropertyFeildName.c_str());
				}
			}
			else if (!xmlStrcmp(name, BAD_CAST"PropertyPosType"))
			{
				char*out = (char*)xmlTextReaderReadString(reader);
				if (out)
				{
					int leftc = TrimLeft((char*)out);
					int rightc = TrimRight((char*)out);
					int len = strlen((char*)out);
					PropertyPosType = (char*)out;
					PropertyPosType = PropertyPosType.substr(leftc, len - rightc - leftc);
				}
			}
			else if (!xmlStrcmp(name, BAD_CAST"PropertyFeildType"))
			{
				char*out = ReadAttrAndConvert(reader, "Row");
				if (out)
				{
					Row = atoi((char*)out);
				}

				out = ReadAttrAndConvert(reader, "Col");
				if (out)
				{
					Col = atoi((char*)out);
				}

				compount = Row*Col;

				out = (char*)xmlTextReaderReadString(reader);
				if (out)
				{
					int leftc = TrimLeft((char*)out);
					int rightc = TrimRight((char*)out);
					int len = strlen((char*)out);
					PropertyFeildType = (char*)out;
					PropertyFeildType = PropertyFeildType.substr(leftc, len - rightc - leftc);
				}


			}

			if ((!xmlStrcmp(name, (const xmlChar *)"Values")))
			{
				vtkDataArray*curdata = ReadDataArray(reader);
				if (curdata)
					data = curdata;


				if (pVtkObj&&data && data->GetSize())
				{
					data->SetNumberOfComponents(compount);
					data->SetNumberOfTuples(data->GetSize() / compount);


					vtkObject* mDataSet = pVtkObj;

					if (mDataSet->GetClassName() == "GMPolyData")
					{
						GMPolyData* DataSet = (GMPolyData*)mDataSet;

						if (PropertyPosType == "VERTEX")
							DataSet->GetPointData()->AddArray(data);

						if (PropertyPosType == "FACE" || PropertyPosType == "VOLUME")
							DataSet->GetCellData()->AddArray(data);
					}
					if (mDataSet->GetClassName() == "GMStructuredPoints")
					{
						GMStructuredPoints* DataSet = (GMStructuredPoints*)mDataSet;

						if (PropertyPosType == "VERTEX")
							DataSet->GetPointData()->AddArray(data);

						if (PropertyPosType == "FACE" || PropertyPosType == "VOLUME")
							DataSet->GetCellData()->AddArray(data);
					}

					if (mDataSet->GetClassName() == "GMUnstructuredGrid")
					{
						GMUnstructuredGrid* DataSet = (GMUnstructuredGrid*)mDataSet;

						if (PropertyPosType == "VERTEX")
							DataSet->GetPointData()->AddArray(data);

						if (PropertyPosType == "FACE" || PropertyPosType == "VOLUME")
							DataSet->GetCellData()->AddArray(data);
					}

					if (mDataSet->GetClassName() == "GMCornerPointGrid")
					{
						GMCornerPointGrid* DataSet = (GMCornerPointGrid*)mDataSet;
						DataSet->AddCellPropertyData(data);

					}
				}
			}

		}
		else if (xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
		{
			if (!xmlStrcmp(name, BAD_CAST"GeoDiscreteCoverage"))
			{


				return 0;
			}
			/*else if (!xmlStrcmp(name,BAD_CAST"GeoPropertyRange"))
			{
				return 0;
			}*/
		}

		nRet = xmlTextReaderRead(reader);
	}

	return 0;
}