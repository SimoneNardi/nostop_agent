#include "WorldMap.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

WorldMap::WorldMap(std::vector<int8_t> const& data_) : m_data(data_) 
{}

std::vector<int8_t> WorldMap::getMap() const 
{
  return m_data;
  
}

void WorldMap::update(std::vector<int8_t> const& data_) 
{
  m_data = data_;
}