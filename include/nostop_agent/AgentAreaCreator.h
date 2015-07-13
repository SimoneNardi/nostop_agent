////////////////////////////////////////////////////
//	AgentAreaCreator.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_AREA_CREATOR_H
#define AGENT_AREA_CREATOR_H
#pragma once

#include "area.h"

#include "nostop_agent/AreaData.h"

#include <vector>

namespace Robotics 
{
	namespace GameTheory
	{
		class AgentAreaCreator
		{
			std::vector<IDSReal2D> m_external;
			std::vector< std::vector<IDSReal2D> > m_internal;

		public:
			AgentAreaCreator();
			
			AgentAreaCreator(nostop_agent::ShapeData external_, std::vector<nostop_agent::ShapeData> internal_);

		public:      
			AreaPtr getArea() const;
		};

	}
}


#endif // CREATE_AREA_H