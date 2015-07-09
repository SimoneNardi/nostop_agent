#include "AgentMotorControl.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

#include "agent.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
AgentMotorControl::AgentMotorControl(std::shared_ptr<Agent> agent_)
  : m_agent(agent_)
  , m_node()
{
  if(m_agent)
  {
    std::stringstream l_agentname;
    l_agentname << "AgentMotorControl_";
    l_agentname << m_agent->getID();
    
    m_controlPub = m_node.advertise<geometry_msgs::Twist>(l_agentname.str().c_str(), 10);
  }
}

/////////////////////////////////////////////
AgentMotorControl::~AgentMotorControl()
{}

/////////////////////////////////////////////
void AgentMotorControl::run()
{
  ros::Rate loop_rate(50);
    
  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    double l_angular = m_agent->getCurrentRotation();
    double l_linear = m_agent->getCurrentSpeed();
    msg.linear.x = l_linear;
    msg.angular.z = l_angular;
        
    m_controlPub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

}