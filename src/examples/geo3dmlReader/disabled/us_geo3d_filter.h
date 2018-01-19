///////////////////////////////////////////////////////////////////////////
//
//  This source file is part of Uniscope Virtual Globe
//  Copyright (c) 2008-2009 by The Uniscope Team . All Rights Reserved
//
///////////////////////////////////////////////////////////////////////////
//
//  Filename: us_xml_filter.h
//  Author  : Uniscope Team 
//  Modifier: Uniscope Team  
//  Created : 
//  Purpose : xml_filter class
//	Reference : 
//
///////////////////////////////////////////////////////////////////////////
#ifndef _US_GEO3D_FILTER_H_
#define _US_GEO3D_FILTER_H_

namespace uniscope_globe
{
	class LIB_FILTER_API geo3d_filter
		: public filter_base
	{
	public: 
		geo3d_filter( void ) { }

		virtual ~geo3d_filter( void ) { }

	public:
		virtual object_base* parse( raw_buffer* in_buffer );
		 
	};
}

#endif // _US_GEO3D_FILTER_H_