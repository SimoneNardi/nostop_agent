#include "MonitorReceiver.h"
#include "WorldMap.h"

#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
MonitorReceiver::MonitorReceiver() 
: m_data(nullptr)
{
	m_data = std::make_shared<WorldMap>();
	m_sub = m_node.subscribe<nav_msgs::OccupancyGrid>("MonitorUpdate", 1, &MonitorReceiver::UpdateMonitorCallBack, this);
}

/////////////////////////////////////////////
MonitorReceiver::~MonitorReceiver()
{}

/////////////////////////////////////////////
void MonitorReceiver::UpdateMonitorCallBack(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  m_data->update(msg->data);
}

/////////////////////////////////////////////
std::shared_ptr<WorldMap> MonitorReceiver::getData() const
{
  return m_data;
}
