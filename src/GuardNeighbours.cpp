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
{
	m_data = std::make_shared<WorldMap>();
	m_sub = m_node.subscribe<nav_msgs::OccupancyGrid>("NeighboursUpdate", 1, &GuardNeighbours::UpdateNeighboursCallBack, this);
}

/////////////////////////////////////////////
GuardNeighbours::~GuardNeighbours()
{}

/////////////////////////////////////////////
void GuardNeighbours::UpdateNeighboursCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	m_data->update(msg->data);
}

/////////////////////////////////////////////
std::shared_ptr<WorldMap> GuardNeighbours::getData() const
{
  return m_data;
}
