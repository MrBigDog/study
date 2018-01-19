///////////////////////////////////////////////////////////////////////////
//
//  This source file is part of Uniscope Virtual Globe
//  Copyright (c) 2008-2009 by The Uniscope Team . All Rights Reserved
//
///////////////////////////////////////////////////////////////////////////
//
//  Filename: us_xml_filter.cpp
//  Author  : Uniscope Team 
//  Modifier: Uniscope Team  
//  Created : 
//  Purpose : xml_filter class
//	Reference : 
//
///////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "us_geo3d_filter.h"
#include "Geo3DML_SDK/GM_XML/Geo3DProject.h"
#include "Geo3DML_SDK/GM_XML/GeoModel.h"
#include "Geo3DML_SDK/GM_XML/FeatureClass.h"
#include "Geo3DML_SDK/GM_XML/GeologicFeature.h"
#include "Geo3DML_SDK/GM_XML/MappedFeature.h"
#include "Geo3DML_SDK/vtkExtending/vtkObject.h"
#include "Geo3DML_SDK/vtkExtending/GMPolyData.h"
#include "Geo3DML_SDK/vtkExtending/vtkCellArray.h"
#include "Geo3DML_SDK/GMXMLFileIO\Geo3DProjectXMLReader.h"

namespace uniscope_globe
{
	void construct_single_mesh(document_base* in_doc, d3d9_hardware_mesh* in_hardware_mesh, gmml::GeologicFeature* pFC)
	{
		in_hardware_mesh->set_name(string_ext::to_wstring(pFC->getName()).c_str());

		in_hardware_mesh->m_fvf = USFVF_XYZ+USFVF_NORMAL;
		//需要加入法线,否则分不开
		in_hardware_mesh->m_vertex_stride = 6*sizeof(float);//包含法线

		std::map<std::string, std::pair<int, double>> mapExtend;
		mapExtend.insert(std::make_pair("XMin", std::make_pair(-1, 0.0)));
		mapExtend.insert(std::make_pair("YMin", std::make_pair(-1, 0.0)));
		mapExtend.insert(std::make_pair("ZMin", std::make_pair(-1, 0.0)));
		mapExtend.insert(std::make_pair("XMax", std::make_pair(-1, 0.0)));
		mapExtend.insert(std::make_pair("YMax", std::make_pair(-1, 0.0)));
		mapExtend.insert(std::make_pair("ZMax", std::make_pair(-1, 0.0)));
		for (int i = 0; i < pFC->GetAttributeValueCount(); ++i)
		{
			AttributeValue attr = pFC->GetAttributeValue(i);
			std::map<std::string, std::pair<int, double>>::iterator iterF = mapExtend.find(attr.fieldName);
			if (mapExtend.end() == iterF)
			{
				continue;
			}
			iterF->second.first = i;
			iterF->second.second = atof(attr.fieldValue.c_str());
		}

		vector3<double> v_min(mapExtend["XMin"].second, mapExtend["YMin"].second, mapExtend["ZMin"].second);
		vector3<double> v_max(mapExtend["XMax"].second, mapExtend["YMax"].second, mapExtend["ZMax"].second);
		in_hardware_mesh->m_mesh_aabb.set_extent(v_min, v_max);
		in_hardware_mesh->set_bound_box(in_hardware_mesh->m_mesh_aabb);

		// vertex [1/4/2018 mahc]
		gmml::MappedFeature* mapped_feature = pFC->GetMappedFeature(0);
		vtkObject* pShape = mapped_feature->GetShape();

		std::string strShapeName = pShape->GetClassName();

		if (strShapeName == "GMPolyData")
		{
			GMPolyData* poly_data = (GMPolyData*)pShape;
			if (poly_data && poly_data->GetGeometryType() == 5)
			{
				vtkPoints* points = poly_data->GetPoints();
				int vCount = points->GetNumberOfPoints();
				in_hardware_mesh->m_num_of_vertices = vCount*2;
				in_hardware_mesh->m_vertex_buffer_size = in_hardware_mesh->m_vertex_stride * in_hardware_mesh->m_num_of_vertices;
				in_hardware_mesh->m_vertex_buffer = new unsigned char[in_hardware_mesh->m_vertex_buffer_size];

				// vertex
				int vertex_offset = 0;
				for (ulong i = 0; i < vCount; i++)
				{
					double xx[3];
					points->GetPoint(i, xx);
					in_hardware_mesh->set_vector3(vertex_offset, i, xx);
					in_hardware_mesh->set_vector3(vertex_offset, i+vCount, xx);
				}

				vtkCellArray* line = poly_data->GetPolys();
				long tCount = line->GetNumberOfCells();
				// 显示双面 [1/4/2018 mahc]
				in_hardware_mesh->m_num_of_triangles = tCount*2;

				if (tCount*2 > 1 << 15)
				{
					in_hardware_mesh->m_32bit_index = true;
					in_hardware_mesh->m_index_buffer = new ulong[in_hardware_mesh->m_num_of_triangles * 3];
					in_hardware_mesh->m_index_buffer_size = sizeof(ulong) * in_hardware_mesh->m_num_of_triangles * 3;
					ulong* v_buffer = (ulong*)in_hardware_mesh->m_index_buffer;
					int v_off = 0;
					for (int i = 0; i < tCount * 4; i = i + 4)
					{
						vtkIdType counta;
						vtkIdType *pts;
						line->GetCell(i, counta, pts);

						v_buffer[v_off] = pts[0];
						v_buffer[v_off + 1] = pts[1];
						v_buffer[v_off + 2] = pts[2];
						v_off += 3;

						v_buffer[v_off] = pts[0]+vCount;
						v_buffer[v_off + 1] = pts[2] + vCount;
						v_buffer[v_off + 2] = pts[1] + vCount;
						v_off += 3;
					}


					// 计算法线 [1/5/2018 mahc]
					std::vector<std::pair<vector_3f, int>> vNormals;
					vNormals.resize(in_hardware_mesh->m_num_of_vertices, std::make_pair(vector_3f(), 0));
					for (ulong i = 0; i < in_hardware_mesh->m_num_of_triangles; ++i)
					{
						int u1 = v_buffer[i * 3], u2 = v_buffer[i * 3 + 1], u3 = v_buffer[i * 3 + 2];
						vector_3f p1 = in_hardware_mesh->get_vector3(0, u1);
						vector_3f p2 = in_hardware_mesh->get_vector3(0, u2);
						vector_3f p3 = in_hardware_mesh->get_vector3(0, u3);
						vector_3f vNor = (p2 - p1).cross(p3 - p2);
						vNormals[u1].first = vNor; vNormals[u1].second++;
						vNormals[u2].first = vNor; vNormals[u2].second++;
						vNormals[u3].first = vNor; vNormals[u3].second++;
					}
					vertex_offset = 3 * sizeof(float);
					for (ulong i = 0; i < vNormals.size(); ++i)
					{
						vNormals[i].first /= vNormals[i].second;
						in_hardware_mesh->set_vector3(vertex_offset, i, &vNormals[i].first);
					}
				}
				else
				{
					in_hardware_mesh->m_index_buffer = new ushort[in_hardware_mesh->m_num_of_triangles * 3];
					ushort* v_buffer = (ushort*)in_hardware_mesh->m_index_buffer;
					in_hardware_mesh->m_index_buffer_size = sizeof(ushort) * in_hardware_mesh->m_num_of_triangles * 3;
					int v_off = 0;
					for (int i = 0; i < tCount * 4; i = i + 4)
					{
						vtkIdType counta;
						vtkIdType *pts;
						line->GetCell(i, counta, pts);

						v_buffer[v_off] = pts[0];
						v_buffer[v_off + 1] = pts[1];
						v_buffer[v_off + 2] = pts[2];
						v_off += 3;

						v_buffer[v_off] = pts[0] + vCount;
						v_buffer[v_off + 1] = pts[2] + vCount;
						v_buffer[v_off + 2] = pts[1] + vCount;
						v_off += 3;
					}

					// 计算法线 [1/5/2018 mahc]
					std::vector<std::pair<vector_3f, int>> vNormals;
					vNormals.resize(in_hardware_mesh->m_num_of_vertices,std::make_pair(vector_3f(),0));
					for (ulong i=0;i<in_hardware_mesh->m_num_of_triangles;++i)
					{
						int u1 = v_buffer[i * 3], u2 = v_buffer[i * 3 + 1], u3 = v_buffer[i * 3 + 2];
						vector_3f p1=in_hardware_mesh->get_vector3(0, u1);
						vector_3f p2 = in_hardware_mesh->get_vector3(0, u2);
						vector_3f p3 = in_hardware_mesh->get_vector3(0, u3);
						vector_3f vNor = (p2 - p1).cross(p3 - p2);
						vNormals[u1].first = vNor; vNormals[u1].second++;
						vNormals[u2].first = vNor; vNormals[u2].second++;
						vNormals[u3].first = vNor; vNormals[u3].second++;
					}
					vertex_offset = 3*sizeof(float);
					for (ulong i=0;i<vNormals.size();++i)
					{
						vNormals[i].first /= vNormals[i].second;
						vNormals[i].first.normalize();
						in_hardware_mesh->set_vector3(vertex_offset, i, &vNormals[i].first);
					}
				}




				resource_manager<ustring>* v_texture_manager = in_doc->get_common_manager_group()->get_texture_manager(L"district_texture");
				in_hardware_mesh->set_texture_manager(v_texture_manager);
			}

		}
	}

	object_base* geo3d_filter::parse( raw_buffer* in_buffer )
	{
		square_buffer* buf = (square_buffer*)in_buffer;
		
		Geo3DProjectXMLReader xmlreader;

		gmml::Geo3DProject* project = NULL;
		
		if (!xmlreader.ReadFile(string_ext::from_wstring(buf->get_path()).c_str(), ""))
		{
			return nullptr;
		}
		

		project = (gmml::Geo3DProject*)xmlreader.GetGeoProject();

		int model_count = project->GetGeoModelCount();
		if (model_count < 1) return nullptr;

		// 先解析第一个 [1/3/2018 mahc]
		gmml::GeoModel* pModel = project->GetGeoModel(0);
		if (nullptr == pModel)
		{
			return nullptr;
		}
		int fc_count = pModel->GetGeoFeatureClassCount();
		gmml::GeologicFeatureClass* pFCls = pModel->GetGeoFeatureClass(0);
		gmml::GeologicFeature* pFC = pFCls->GetGeologicFeature(0);
		
		d3d9_hardware_mesh* v_sub_mesh = new d3d9_hardware_mesh();
		v_sub_mesh->m_path = buf->get_path();
		construct_single_mesh(buf->get_document(), v_sub_mesh, pFC);

		// 需要释放 [1/3/2018 mahc]
		AUTO_DELETE(project);
		return v_sub_mesh;

		//filter_buffer* buf = (filter_buffer*)in_buffer;
		
		//xml_parser_base* v_parser = singleton_xml_parser_manager::instance().get_parser( L"XML" );

		//// load xml buffer
		//TiXmlDocument xml_doc;
		//xml_doc.Parse( (const char *)buf->get_buffer() );

		//// parse
		//TiXmlElement *v_elem = xml_doc.RootElement();
		//return v_parser->forward_parse( buf->m_container, v_elem, buf->m_parent );

		return nullptr;
	}
}