#include "AgentProcess.h"

#include "StateUpdater.h"
#include "MotorControl.h"


#include "iAgent.h"
#include "nostop_agent/GuardSensorCtrl.h"
#include "nostop_agent/PlayerNotifyStatus.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////////
void AgentProcess::setRobotName(std::string name_)
{
    m_agent->setName(name_);
}

/////////////////////////////////////////////
void AgentProcess::init()
{
  m_stateUpdater = std::make_shared<StateUpdater>( m_agent );
  m_stateUpdater->start();
    
  m_motorControl = std::make_shared<MotorControl>( m_agent );
  m_motorControl->start();
  
  m_notifyStatus = m_node.serviceClient<nostop_agent::PlayerNotifyStatus>("NotifyStatus");
}

/////////////////////////////////////////////
void AgentProcess::setKinectLocalizer()
{
  m_agent->setKinectLocalizer();
}

/////////////////////////////////////////////
void AgentProcess::setSimulatorLocalizer()
{
  m_agent->setSimulatorLocalizer();
}

/////////////////////////////////////////////
void AgentProcess::reachTargetConfiguration()
{
  // StateUpdater is computing configuration to reach the target,
  // it is necessary to chek only if agent is arrived to target
  while( m_agent->getStatus() != Agent::STANDBY )
    ros::spinOnce();
  
  //TODO non deve girare a vuoto, d3eve rimanere in attese di una condition variable!
}

//////////////////////////////////////////////////
// send a broadcast message of unemployed agents
bool AgentProcess::notifyStatus()
{
  nostop_agent::PlayerNotifyStatus l_srv;
  l_srv.request.id = m_agent->getID();
  if ( !m_notifyStatus.call(l_srv) )
  {
      ROS_ERROR("Failed to call service Notify Status");
      return false;
  }

  return true;
}
