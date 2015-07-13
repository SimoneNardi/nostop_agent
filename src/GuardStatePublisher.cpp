#include "GuardStatePublisher.h"

#include "nostop_agent/GuardStateData.h"

#include "ros/ros.h"

#include "guard.h"
#include "agent.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
GuardStatePublisher::GuardStatePublisher(std::shared_ptr<Guard> agent_) 
  : m_guard(agent_)
  , m_node()
{
  if(m_guard)
  {
    std::stringstream l_guardname;
    l_guardname << "GuardState_";
    l_guardname << m_guard->getID();
    
    m_statePub = m_node.advertise<nostop_agent::GuardStateData>(l_guardname.str().c_str(), 10);
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
    nostop_agent::GuardStateData msg;

    msg.id = m_guard->getID();
    
    msg.msg = "GuardState";
    
    AgentPosition l_agentPos = m_guard->getCurrentPosition();
    IDSReal2D l_point = l_agentPos.getPoint2D();
    msg.x = l_point(0);
    msg.y = l_point(1);
    
    AgentPosition l_previousPos = m_guard->getPreviousPosition();
    IDSReal2D l_prev_point = l_previousPos.getPoint2D();
    IDSReal2D l_delta = l_point - l_prev_point;
    
    msg.heading = IDSMath::polarPhi2D(l_delta(0), l_delta(1));
        
    CameraPosition l_camera = l_agentPos.getCameraControl();
    
    msg.min_radius = l_camera.getNearRadius();
    msg.max_radius = l_camera.getFarRadius();
    msg.camera_heading = l_camera.getOrientation();
    msg.view = l_camera.getAngleOfView();
    
    msg.status = m_guard->getStatus();
    
    m_statePub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

}