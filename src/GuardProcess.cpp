#include "GuardProcess.h"
#include "GuardStatePublisher.h"

#include "Conversions.h"

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
  std::shared_ptr<iGuard> l_guard = std::static_pointer_cast<iGuard>(m_agent);
  
  std::shared_ptr<LearningInitializer> l_learning = l_guard->getLearningInitializer();
  std::shared_ptr<iLocalizer> l_localizer = l_guard->getLocalizer();
  
  // learning initialization:
  bool l_learning_is_ready = l_learning->isLearningInitialized();
  
  // initial position:
  bool l_position_is_ready = l_localizer->isLocalizerInitialized();

  if (l_learning_is_ready && l_position_is_ready)
  {
      int l_id = l_learning->getID();

      geometry_msgs::Point l_geomPoint = l_guard->getCurrentConfigurationPosition();
      IDSReal2D l_point = Conversions::Point2IDSReal2D(l_geomPoint);
      
      nostop_agent::GuardSensorCtrl l_guardSensorCtrl = l_guard->getCameraControl();
      CameraPosition l_cameraPos  = Conversions::GuardSensorCtrl2CameraPosition(l_guardSensorCtrl);
      AgentPosition l_agentPos (l_point, l_cameraPos);
      
      std::shared_ptr<Guard> l_LGuard = std::make_shared<Guard>(1, l_id, l_agentPos, 1, 2);
    
      l_guard->setLearnGuard(l_LGuard);
  }
  
  return l_ready;
}
