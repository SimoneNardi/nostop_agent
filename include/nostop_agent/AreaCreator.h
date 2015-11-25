////////////////////////////////////////////////////
//	AreaCreator.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AREA_CREATOR_H
#define AREA_CREATOR_H
#pragma once

#include "area.h"

#include <nostop_area/Shape.h>

#include <vector>

namespace Robotics 
{
	namespace GameTheory
	{
		class AreaCreator
		{
			std::vector<Real2D> m_external;
			std::vector< std::vector<Real2D> > m_internal;

		public:
			AreaCreator();
			
			AreaCreator(nostop_area::Shape external_, std::vector<nostop_area::Shape> internal_);

		public:      
			AreaPtr getArea() const;
		};

	}
}


#endif // CREATE_AREA_H