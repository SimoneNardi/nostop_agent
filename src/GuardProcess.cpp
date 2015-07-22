#include "GuardProcess.h"
#include "StateUpdater.h"
#include "StatePublisher.h"
#include "MotorControl.h"

#include "ros/ros.h"
#include <sstream>

#include "iGuard.h"
#include <memory>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
GuardProcess::GuardProcess(std::string name_)
: AgentProcess()
{
  m_agent = std::make_shared<iGuard>();
  m_agent->setName(name_);
  
  m_motorControl = std::make_shared<MotorControl>( m_agent );
  m_motorControl->start();
    
  std::shared_ptr<iGuard> l_guard = std::static_pointer_cast<iGuard>(m_agent);
  
  m_stateUpdater = std::make_shared<StateUpdater>( l_guard );
  m_stateUpdater->start();
  
  m_statePublisher = std::make_shared<StatePublisher>( l_guard );
  m_statePublisher->start();
}

/////////////////////////////////////////////
GuardProcess::~GuardProcess()
{}

/////////////////////////////////////////////
AgentPosition computeAgentPosition(geometry_msgs::Point & point_)
{
  // TODO
  return AgentPosition();
}