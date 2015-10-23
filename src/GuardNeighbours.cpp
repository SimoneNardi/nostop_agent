#include "GuardNeighbours.h"
#include "WorldMap.h"

#include "ros/ros.h"

#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
GuardNeighbours::GuardNeighbours() 
: m_data(nullptr)
, m_isUpdated(false)
{
	m_data = std::make_shared<WorldMap>();
	m_sub = m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/neighbours", 1, &GuardNeighbours::UpdateNeighboursCallBack, this);
}

/////////////////////////////////////////////
GuardNeighbours::~GuardNeighbours()
{}

/////////////////////////////////////////////
void GuardNeighbours::UpdateNeighboursCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	Lock1 lock(m_mutex);
	m_time = msg->info.map_load_time;
	m_data->update(msg->data);
	
	this->setNew();
	// notify new data
}

/////////////////////////////////////////////
std::shared_ptr<WorldMap> GuardNeighbours::getData() const
{
  Lock1 lock(m_mutex);
  return m_data;
}
