#include "GuardInterface.h"
#include "GuardStateUpdater.h"
#include "GuardStatePublisher.h"
#include "AgentMotorControl.h"

#include "ros/ros.h"
#include <sstream>

#include "guard.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
GuardInterface::GuardInterface(int id_, AgentPosition initial_position_)
: AgentInterface()
{
  m_agent = std::make_shared<Guard>(1, id_, initial_position_);

  m_stateUpdater = std::make_shared<GuardStateUpdater>( std::dynamic_pointer_cast<Guard>(m_agent) );
  m_stateUpdater->start();
  
  m_statePublisher = std::make_shared<GuardStatePublisher>( std::dynamic_pointer_cast<Guard>(m_agent) );
  m_statePublisher->start();
  
  m_motorControl = std::make_shared<AgentMotorControl>( m_agent );
  m_motorControl->start();
}

/////////////////////////////////////////////
GuardInterface::~GuardInterface()
{}

/////////////////////////////////////////////
AgentPosition computeAgentPosition(geometry_msgs::Point & point_)
{
  // TODO
  return AgentPosition();
}

/////////////////////////////////////////////
IDSQuaternion computeAgentOrientation(geometry_msgs::Quaternion & orientation_)
{
  // TODO
  return IDSQuaternion();
}