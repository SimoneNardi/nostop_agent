#include "AgentProcess.h"

#include "StateUpdater.h"
#include "MotorControl.h"
#include "iAgent.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
AgentProcess::AgentProcess() 
{}

/////////////////////////////////////////////
AgentProcess::~AgentProcess()
{}

//////////////////////////////////////////////////
void AgentProcess::setRobotName(std::string name_)
{
    m_agent->setName(name_);
}

/////////////////////////////////////////////
void AgentProcess::start()
{
  m_stateUpdater = std::make_shared<StateUpdater>( m_agent );
  m_stateUpdater->start();
    
  //m_motorControl = std::make_shared<MotorControl>( m_agent );
  //m_motorControl->start();
}

/////////////////////////////////////////////
bool AgentProcess::setKinectLocalizer()
{
   return m_agent->setKinectLocalizer();
}

/////////////////////////////////////////////
bool AgentProcess::setSimulatorLocalizer()
{
  return m_agent->setSimulatorLocalizer();
}