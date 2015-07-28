#include "StateUpdater.h"

#include "geometry_msgs/Pose.h"

#include "ros/ros.h"

#include "iAgent.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
StateUpdater::StateUpdater(std::shared_ptr<iAgent> agent_) 
  : ThreadBase()
  , m_agent(agent_)
{}

/////////////////////////////////////////////
StateUpdater::~StateUpdater()
{}

/////////////////////////////////////////////
void StateUpdater::run()
{
  ros::Rate loop_rate(10);
    
  iLocalizerPtr l_localizer = m_agent->getLocalizer();
  
  int count = 0;
  while (ros::ok())
  {
    Configuration l_currentConfig = m_agent->getCurrentConfiguration();
    
    if ( l_localizer->isInitialized() && l_localizer->isUpdated() )
    {
      l_currentConfig = l_localizer->getConfiguration();
    } 
      
    geometry_msgs::Point l_point_ = l_currentConfig.getPosition();
    m_agent->updateCurrentPosition( l_point_ );
    geometry_msgs::Quaternion l_orientation_ = l_currentConfig.getOrientation();
    m_agent->updateCurrentOrientation( l_orientation_ );
    
    m_agent->computeConfigurationToTarget();
    
    Configuration l_newConfig = m_agent->getCurrentConfiguration();

    m_agent->setCurrentConfiguration( l_newConfig );
    
    if ( m_agent->isArrived() )
      m_agent->setStandByStatus();
      
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

}