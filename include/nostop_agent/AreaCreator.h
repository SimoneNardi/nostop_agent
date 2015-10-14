////////////////////////////////////////////////////
//	AreaCreator.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AREA_CREATOR_H
#define AREA_CREATOR_H
#pragma once

#include "area.h"

#include <nostop_agent/Shape.h>

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
			
			AreaCreator(nostop_agent::Shape external_, std::vector<nostop_agent::Shape> internal_);

		public:      
			AreaPtr getArea() const;
		};

	}
}


#endif // CREATE_AREA_H