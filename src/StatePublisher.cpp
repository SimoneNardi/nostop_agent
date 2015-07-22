#include "StatePublisher.h"

#include "nostop_agent/GuardStateData.h"

#include "ros/ros.h"

#include "iGuard.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
StatePublisher::StatePublisher(std::shared_ptr<iGuard> agent_) 
  : m_guard(agent_)
  , m_node()
{
  if(m_guard)
  {
     std::stringstream l_guardName;
     l_guardName << "StatePublisher_";
     l_guardName << m_guard->getName();
    
     m_statePub = m_node.advertise<nostop_agent::GuardStateData>(l_guardName.str().c_str(), 10);
  }
}

/////////////////////////////////////////////
StatePublisher::~StatePublisher()
{}


/////////////////////////////////////////////
void StatePublisher::run()
{
  ros::Rate loop_rate(50);
    
  int count = 0;
  while (ros::ok())
  {
    nostop_agent::GuardStateData msg;

    msg.id = m_guard->getID();
    
    msg.msg = "GuardState";
    
    Configuration l_currentPose =  m_guard->getCurrentConfiguration();
    
    AgentPosition l_agentPos = m_guard->getCurrentAgentPosition();
    IDSReal2D l_point = l_agentPos.getPoint2D();
    msg.x = l_point(0);
    msg.y = l_point(1);
    
//     AgentPosition l_previousPos = m_guard->getPreviousPosition();
//     IDSReal2D l_prev_point = l_previousPos.getPoint2D();
//     IDSReal2D l_delta = l_point - l_prev_point;
//     msg.heading = IDSMath::polarPhi2D(l_delta(0), l_delta(1));
//     msg.heading = l_currentPose.getOrientation(); 
    
    geometry_msgs::Quaternion l_orientation = l_currentPose.getOrientation();
    tf::Pose l_pose;
    tf::poseMsgToTF(l_currentPose.getPose(), l_pose);
    msg.heading = tf::getYaw(l_pose.getRotation());
            
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