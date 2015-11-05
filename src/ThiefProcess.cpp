#include "ThiefProcess.h"
#include "ThiefStatePublisher.h"
#include "iThief.h"

#include "AgentProcess.h"

#include "ros/ros.h"

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
void ThiefProcess::start()
{
  AgentProcess::start();
  
  m_statePublisher = std::make_shared<ThiefStatePublisher>( m_agent );
  m_statePublisher->start();
}

/////////////////////////////////////////////
ThiefProcess::~ThiefProcess()
{}

//////////////////////////////////////////////////
bool ThiefProcess::isReady()
{
  return true;
}

