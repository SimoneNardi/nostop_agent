#include "StateUpdater.h"

#include "geometry_msgs/Pose.h"

#include "ros/ros.h"

#include "iAgent.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

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
  int standby_count = 0;
  while (ros::ok())
  {
    if (m_agent->getStatus() != Agent::STANDBY)
    {
      Configuration l_currentConfig = m_agent->getCurrentConfiguration();
      
      if ( l_localizer->isInitialized() && l_localizer->isUpdated() )
      {
	l_currentConfig = l_localizer->getConfiguration();
      }
	
      geometry_msgs::Point l_point = l_currentConfig.getPosition();
      m_agent->updateCurrentPosition( l_point );
      geometry_msgs::Quaternion l_orientation = l_currentConfig.getOrientation();
      m_agent->updateCurrentOrientation( l_orientation );
      
      m_agent->computeConfigurationToTarget();
      
      Configuration l_newConfig = m_agent->getCurrentConfiguration();
      //m_agent->setCurrentConfiguration( l_newConfig );
      m_agent->setCurrentConfiguration( l_newConfig );
      
      if ( m_agent->isArrived() )
      {
	m_time = ros::Time::now();
	// set status and notify to Simulator
	m_agent->setStandByStatus();
	standby_count=0;
      }
      
      geometry_msgs::Pose l_pose = l_newConfig.getPose();
      tf::Transform l_transform;
      l_transform.setOrigin( tf::Vector3(l_pose.position.x, l_pose.position.y, l_pose.position.z) );
      
      // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
      tf::Quaternion l_quat;
      tf::quaternionMsgToTF(l_pose.orientation, l_quat);
      l_transform.setRotation( l_quat );
      m_broadcaster.sendTransform(tf::StampedTransform(l_transform, ros::Time::now(), "world", m_agent->getName()));
    }
    else
    {
      ++standby_count;
    }
    
    if (standby_count == 20)
    {
      if ( m_agent->isArrived() )
      {
	m_time = ros::Time::now();
	// set status and notify to Simulator
	m_agent->setStandByStatus();
	standby_count=0;
      }
    }
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

}