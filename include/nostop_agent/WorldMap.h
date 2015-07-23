////////////////////////////////////////////////////
//	WorldMap.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef WORLD_MAP_H
#define WORLD_MAP_H
#pragma once

#include <vector>
#include <stdint.h>

namespace Robotics
{
	namespace GameTheory
	{
		class WorldMap
		{
		protected:
		    std::vector<int8_t> m_data;
		public:
		    WorldMap(std::vector<int8_t> const& data_ = std::vector<int8_t>());
		    std::vector<int8_t> getMap() const;
		    void update(std::vector<int8_t> const& data_);
		};
	}
}


#endif // WORLD_MAP_H