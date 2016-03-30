#include "ThiefStatePublisher.h"

#include "nostop_agent/ThiefState.h"

#include "ros/ros.h"

#include "iThief.h"

#include <nav_msgs/Odometry.h>

#include "GlobalPreProcessor.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
ThiefStatePublisher::ThiefStatePublisher(std::shared_ptr<iAgent> agent_) 
: StatePublisher(agent_)
{
  if(m_agent)
  {
     m_thief = std::static_pointer_cast<iThief>(m_agent);
     if(m_thief)
      {
	std::stringstream l_thiefName;
	l_thiefName << "/publisher/state/thief/";
	l_thiefName << m_thief->getID();
		
#ifdef _DEBUG_PRINT
	std::cout << "*** Advertise ThiefStatePublisher " << l_thiefName <<std::endl;
#endif
	m_statePub = m_node.advertise<nostop_agent::ThiefState>(l_thiefName.str().c_str(), 10);
      }
  }
}

/////////////////////////////////////////////
ThiefStatePublisher::~ThiefStatePublisher()
{}


/////////////////////////////////////////////
void ThiefStatePublisher::run()
{
  ros::Rate loop_rate(50);
    
  int count = 0;
  while (ros::ok())
  {
    nostop_agent::ThiefState msg;

    Configuration l_currentPose =  m_agent->getCurrentConfiguration();
    msg.odometry = l_currentPose.getOdometry();
                
    m_statePub.publish(msg);
    
    if( m_agent->isReal() )
      m_posePub.publish(m_agent->getCurrentConfigurationPose());

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}