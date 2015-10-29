#include "AgentProcess.h"

#include "StateUpdater.h"
#include "MotorControl.h"


#include "iAgent.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

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
    
  m_motorControl = std::make_shared<MotorControl>( m_agent );
  //m_motorControl->start();
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