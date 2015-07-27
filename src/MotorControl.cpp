#include "MotorControl.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

#include "iAgent.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
MotorControl::MotorControl(std::shared_ptr<iAgent> agent_)
  : ThreadBase()
  , m_agent(agent_)
  , m_node()
{
  if(m_agent)
  {
    std::stringstream l_agentname;
    l_agentname << "MotorControl_";
    l_agentname << m_agent->getName();
    
    m_controlPub = m_node.advertise<geometry_msgs::Twist>(l_agentname.str().c_str(), 10);
  }
}

/////////////////////////////////////////////
MotorControl::~MotorControl()
{}

/////////////////////////////////////////////
void MotorControl::run()
{
  ros::Rate loop_rate(50);
    
  int count = 0;
  while (ros::ok())
  {
    m_controlPub.publish(m_agent->getCurrentConfigurationTwist());

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

}