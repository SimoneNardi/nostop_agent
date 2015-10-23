#include "GuardProcess.h"
#include "GuardStatePublisher.h"

#include "Conversions.h"

#include "ros/ros.h"

#include "iGuard.h"
#include "LearningInitializer.h"
#include "StateUpdater.h"

#include "guard.h"

#include <sstream>
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
void GuardProcess::start()
{
  AgentProcess::start();
  
  m_statePublisher = std::make_shared<GuardStatePublisher>( m_agent );
  m_statePublisher->start();
  
  std::shared_ptr<iGuard> l_guard = std::static_pointer_cast<iGuard>(m_agent);
  l_guard->startLearning();
}

/////////////////////////////////////////////
GuardProcess::~GuardProcess()
{}

//////////////////////////////////////////////////
void GuardProcess::setRobotAlgorithm(std::string alg_)
{
  std::shared_ptr<iGuard> l_guard = std::static_pointer_cast<iGuard>(m_agent);
  l_guard->setRobotAlgorithm(alg_);
}

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
  // learning initialization:
  bool l_learning_is_ready = l_learning->isInitialized();

  iLocalizerPtr l_localizer = l_guard->getLocalizer();  
  // initial position:
  bool l_position_is_ready = l_localizer->isInitialized();

  if (l_learning_is_ready && l_position_is_ready)
  {
      int l_id = l_learning->getID();
      
      // init della posizione del robot.
      geometry_msgs::Pose l_geomPoint = l_localizer->getConfiguration().getPose();
      l_guard->updateCurrentPose(l_geomPoint);
      Real2D l_point = Conversions::Point2Real2D(l_geomPoint.position);
      
      nostop_agent::GuardSensorCtrl l_initialConfiguration;
      l_initialConfiguration.max_radius = 3;
      l_initialConfiguration.min_radius = 0;
      l_initialConfiguration.fov = 360;
      l_initialConfiguration.heading = 0;

      CameraPosition l_cameraPos  = Conversions::GuardSensorCtrl2CameraPosition(l_initialConfiguration);
      
      AgentPosition l_agentPos (l_point, l_cameraPos);
      
      std::shared_ptr<Guard> l_LGuard = std::make_shared<Guard>(1, l_id, l_agentPos, 1, 2);
      
      l_guard->setGuardPtr(l_LGuard);
      return true;
  }
  
  return false;
}

//////////////////////////////////////////////////
void GuardProcess::createLearningAlgorithm(std::shared_ptr<Area> area_)
{
	std::static_pointer_cast<iGuard>(m_agent)->createLearningAlgorithm(area_);
}
