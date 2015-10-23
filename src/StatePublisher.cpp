#include "StatePublisher.h"

#include "nostop_agent/GuardState.h"
#include "nostop_agent/GuardSensorCtrl.h"

#include "ros/ros.h"

#include "iGuard.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
StatePublisher::StatePublisher(std::shared_ptr<iAgent> agent_) 
: ThreadBase()
, m_agent(agent_)
, m_node()
{
  if(m_agent)
  {
    std::stringstream l_agentPose;
    l_agentPose << "/";
    l_agentPose << m_agent->getName();
    l_agentPose << "/pose";
        
    m_posePub = m_node.advertise<geometry_msgs::Pose>(l_agentPose.str().c_str(), 10);
  }
}

/////////////////////////////////////////////
StatePublisher::~StatePublisher()
{}

/////////////////////////////////////////////
void StatePublisher::run()
{
  ros::Rate loop_rate(5);
  
  int count = 0;
  while ( ros::ok() )
  {
    if( m_agent->isReal() )
      m_posePub.publish(m_agent->getCurrentConfigurationPose());
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}
