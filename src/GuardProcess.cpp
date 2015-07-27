#include "GuardProcess.h"
#include "GuardStatePublisher.h"

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
}
 
/////////////////////////////////////////////
void GuardProcess::init()
{
  AgentProcess::init();
  
  m_statePublisher = std::make_shared<GuardStatePublisher>( m_agent );
  m_statePublisher->start();
}

/////////////////////////////////////////////
GuardProcess::~GuardProcess()
{}

//////////////////////////////////////////////////
void GuardProcess::setCamera(nostop_agent::GuardSensorCtrl & camera_)
{
  // TODO
  std::shared_ptr<iGuard> l_guard = std::static_pointer_cast<iGuard>(m_agent);
  if (l_guard)
  {
	l_guard->setCameraCtrl(camera_);
  }
}

//////////////////////////////////////////////////
bool GuardProcess::isReady()
{
  std::shared_ptr<LearningInitializer> l_learning = (std::static_pointer_cast<iGuard>(m_agent))->getLearningInitializer();
  
  bool l_ready = l_learning->isLearningInitialized();
  
  if (l_ready)
  {
    this->setID( l_learning->getID() );
  }
  
  return l_ready;
}
