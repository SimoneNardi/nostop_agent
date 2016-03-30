#include "GuardStatePublisher.h"

#include "nostop_agent/GuardState.h"
#include "nostop_agent/GuardSensorCtrl.h"

#include "ros/ros.h"

#include "iGuard.h"

#include <nav_msgs/Odometry.h>

#include "GlobalPreProcessor.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
GuardStatePublisher::GuardStatePublisher(std::shared_ptr<iAgent> agent_) 
: StatePublisher(agent_)
{
  if(m_agent)
  {
     m_guard = std::static_pointer_cast<iGuard>(m_agent);
     if(m_guard)
      {
	std::stringstream l_guardState;
	l_guardState << "/publisher/state/guard/";
	l_guardState << m_guard->getID();
	
#ifdef _DEBUG_PRINT
	std::cout << "*** Advertise GuardStatePublisher "<< l_guardState.str() <<std::endl;
#endif
	m_statePub = m_node.advertise<nostop_agent::GuardState>(l_guardState.str().c_str(), 10);
      }
  }
}

/////////////////////////////////////////////
GuardStatePublisher::~GuardStatePublisher()
{}


/////////////////////////////////////////////
void GuardStatePublisher::run()
{
  ros::Rate loop_rate(50);
    
  int count = 0;
  while (ros::ok())
  {
    nostop_agent::GuardState msg;

    Configuration l_currentPose =  m_agent->getCurrentConfiguration();
    msg.odometry = l_currentPose.getOdometry();
                
    msg.sensor = m_guard->getCameraControl();
        
    m_statePub.publish(msg);
    
    if( m_agent->isReal() )
      m_posePub.publish(m_agent->getCurrentConfigurationPose());

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}