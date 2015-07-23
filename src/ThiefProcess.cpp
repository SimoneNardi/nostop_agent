#include "ThiefProcess.h"
#include "StateUpdater.h"
#include "ThiefStatePublisher.h"
#include "MotorControl.h"

#include "ros/ros.h"
#include <sstream>

#include "iThief.h"
#include <memory>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
ThiefProcess::ThiefProcess(std::string name_)
: AgentProcess()
{
  m_agent = std::make_shared<iThief>();
  m_agent->setName(name_);
}

/////////////////////////////////////////////
void ThiefProcess::init()
{
  AgentProcess::init();
  
  m_statePublisher = std::make_shared<ThiefStatePublisher>( m_agent );
  m_statePublisher->start();
}

/////////////////////////////////////////////
ThiefProcess::~ThiefProcess()
{}

//////////////////////////////////////////////////
void ThiefProcess::setCamera(nostop_agent::GuardSensorCtrl & camera_)
{
  // TODO
  std::shared_ptr<iThief> l_guard = std::static_pointer_cast<iThief>(m_agent);
  if (l_guard)
  {
    
  }
}
