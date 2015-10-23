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
, m_isUpdated(false)
{
	m_data = std::make_shared<WorldMap>();
	m_sub = m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/monitor", 1, &MonitorReceiver::UpdateMonitorCallBack, this);
}

/////////////////////////////////////////////
MonitorReceiver::~MonitorReceiver()
{}

/////////////////////////////////////////////
void MonitorReceiver::UpdateMonitorCallBack(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  Lock1 lock(m_mutex);
  m_time = msg->info.map_load_time;
  m_data->update(msg->data);
  
  this->setNew();
  // notify new data
}

/////////////////////////////////////////////
std::shared_ptr<WorldMap> MonitorReceiver::getData() const
{
  Lock1 lock(m_mutex);
  return m_data;
}
