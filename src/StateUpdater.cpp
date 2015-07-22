#include "StateUpdater.h"

#include "geometry_msgs/Pose.h"

#include "ros/ros.h"

#include "guard.h"
#include "agent.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
StateUpdater::StateUpdater(std::shared_ptr<iGuard> agent_) 
  : m_guard(agent_)
{
    
}

/////////////////////////////////////////////
StateUpdater::~StateUpdater()
{}

/////////////////////////////////////////////
void StateUpdater::run()
{
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
      
  ros::Rate loop_rate(10);
    
  int count = 0;
  while (ros::ok())
  {
    current_time = ros::Time::now();
    
    m_guard.updateCurrentConfiguration( m_guard.localizer().getConfiguration() );
    
    Configuration l_config = m_guard.getConfiguration();
        
    geometry_msgs::Point l_point = l_config.getPosition();
    double x = l_point.x;
    double y = l_point.y;
    
    geometry_msgs::Quaternion l_orientation = l_config.getOrientation();
    tf::Pose l_pose;
    tf::poseMsgToTF(l_config.m_odom.pose.pose, l_pose);
    double th = tf::getYaw(l_pose.getRotation());
    
    double vx = l_config.m_odom.twist.twist.linear.x;
    double vy = l_config.m_odom.twist.twist.linear.y;
    double vth = l_config.m_odom.twist.twist.angular.z; 
    
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    m_guard.setCurrentConfiguration( Configuration(odom) );
    
    if ( m_guard.isArrived() )
      m_guard.setStandByStatus();
    
    last_time = current_time;
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

}