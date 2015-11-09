#include "ThiefProcess.h"
#include "ThiefStatePublisher.h"
#include "iThief.h"
#include "thief.h"

#include "AgentProcess.h"

#include "Conversions.h"

#include "ros/ros.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
ThiefProcess::ThiefProcess(std::string const& name_, int id_)
: AgentProcess()
, m_id (id_)
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
  
  std::shared_ptr<iThief> l_thief = std::static_pointer_cast<iThief>(m_agent);
  
  iLocalizerPtr l_localizer = l_thief->getLocalizer();  
  Configuration l_conf = l_localizer->getConfiguration();
  l_thief->setTargetConfiguration( l_conf );
}

/////////////////////////////////////////////
ThiefProcess::~ThiefProcess()
{}

//////////////////////////////////////////////////
bool ThiefProcess::isReady()
{
  std::shared_ptr<iThief> l_thief = std::static_pointer_cast<iThief>(m_agent);
  
  iLocalizerPtr l_localizer = l_thief->getLocalizer();  
  // initial position:
  bool l_position_is_ready = l_localizer->isInitialized();

  if (l_position_is_ready)
  {
      // init della posizione del robot.
      Configuration l_conf = l_localizer->getConfiguration();
      geometry_msgs::Pose l_geomPose = l_conf.getPose();
      l_thief->updateCurrentPose( l_geomPose );
      Real2D l_point = Conversions::Point2Real2D( l_geomPose.position );
      
      CameraPosition l_cameraPos;
      AgentPosition l_agentPos (l_point, l_cameraPos);
      
      std::shared_ptr<Thief> l_LThief = std::make_shared<Thief>(m_id, l_agentPos);
      
      l_thief->setThiefPtr(l_LThief);
      return true;
  }
  
  return false;
}
