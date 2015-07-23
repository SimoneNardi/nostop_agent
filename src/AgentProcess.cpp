#include "AgentProcess.h"

#include "StateUpdater.h"
#include "MotorControl.h"


#include "iAgent.h"
#include <nostop_agent/GuardSensorCtrl.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////////
void AgentProcess::setRobotName(std::string name_)
{
    m_agent->setName(name_);
}

//////////////////////////////////////////////////
void AgentProcess::setRobotAlgorithm(std::string alg_)
{
  // TODO
}

//////////////////////////////////////////////////
void AgentProcess::setID(int id_)
{
  // TODO
}

/////////////////////////////////////////////
void AgentProcess::init()
{
  m_stateUpdater = std::make_shared<StateUpdater>( m_agent );
  m_stateUpdater->start();
    
  m_motorControl = std::make_shared<MotorControl>( m_agent );
  m_motorControl->start();
}