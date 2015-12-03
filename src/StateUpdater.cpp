#include "StateUpdater.h"

#include "geometry_msgs/Pose.h"

#include "ros/ros.h"

#include "iAgent.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

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
    
  int standby_count = 0;
  while (ros::ok())
  {
    if (m_agent->getStatus() != Agent::STANDBY)
    {
      if ( m_agent->isArrived(0.4) )
      {
	m_agent->MoveToNextPosition_LearningAgent();

	// set status and notify to Simulator
	m_agent->setStandByStatus();
      }
      
      standby_count = 0;
    }
    else
    {
      ++standby_count;
    }
    
    if (standby_count > 20)
    {
      if ( m_agent->isArrived(0.4) )
      {
	// set status and notify to Simulator
	m_agent->setStandByStatus();
	standby_count=0;
      }
    }
    
    ros::spinOnce();

    loop_rate.sleep();
  }

}