#include "iLocalizer.h"

#include <nostop_agent/Id_robot.h>

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	Configuration iLocalizer::getConfiguration()
	{
	    Lock lock(m_mutex);
	    m_updated = false;
	    return m_config;
	}

	////////////////////////////////////////////////////
	geometry_msgs::Point iLocalizer::getPosition()
	{
	  Lock lock(m_mutex);
	  m_updated = false;
	  return m_config.getPosition();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Quaternion iLocalizer::getOrientation()
	{
	  Lock lock(m_mutex);
	  m_updated = false;
	  return m_config.getOrientation();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Twist iLocalizer::getTwist()
	{
	  Lock lock(m_mutex);
	  m_updated = false;
	  return m_config.getTwist();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Pose iLocalizer::getPose()
	{
	  Lock lock(m_mutex);
	  m_updated = false;
	  return m_config.getPose();
	}
	
	////////////////////////////////////////////////////
	nav_msgs::Odometry iLocalizer::getOdometry()
	{
	  Lock lock(m_mutex);
	  m_updated = false;
	  return m_config.getOdometry();
	}
	
	////////////////////////////////////////////////////
	void iLocalizer::updatePose(const geometry_msgs::Pose::ConstPtr & pose_)
	{
	  geometry_msgs::Pose l_pose;
	  l_pose.orientation = pose_->orientation;
	  l_pose.position = pose_->position;

	  Lock l_lock(m_mutex);
	  m_updated = true;
	  
 	  m_config.setPose(l_pose);
	  
	  m_initialized = true;
	  
	  m_pub.publish(pose_);
	}
	
	////////////////////////////////////////////////////
	void iLocalizer::subscribeTopic()
	{
	  m_sub = m_node.subscribe<geometry_msgs::Pose>(m_sub_name.c_str(), 1, &iLocalizer::updatePose, this);
	  m_pub = m_node.advertise<geometry_msgs::Pose>(m_pub_name.c_str(), 1);
	 }
	
	////////////////////////////////////////////////////
	bool iLocalizer::isInitialized()
	{
	  Lock l_lock(m_mutex);
	  return m_initialized;
	}
	
	////////////////////////////////////////////////////
	bool iLocalizer::isUpdated()
	{
	  Lock l_lock(m_mutex);
	  return m_updated;
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////
	SimulatorLocalizer::SimulatorLocalizer(std::string name_)
	: iLocalizer(name_)
	{
	  std::string l_agentname = "/";
	  l_agentname += name_;
	  l_agentname += "/localizer/gazebo/pose";
	  m_sub_name = l_agentname;
	  
	  l_agentname = "/";
	  l_agentname += name_;
	  l_agentname += "/localizer/pose";
	  m_pub_name = l_agentname;
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////
	KinectLocalizer::KinectLocalizer(std::string name_) 
	: iLocalizer(name_)
	{
	  std::string l_agentname = "/";
	  l_agentname += name_;
	  l_agentname += "/localizer/kinect/pose";
	  m_sub_name = l_agentname;	  
	  
	  l_agentname = "/";
	  l_agentname += name_;
	  l_agentname += "/localizer/pose";
	  m_pub_name = l_agentname;
	 
	  ros::Publisher l_pub = m_node.advertise<geometry_msgs::Pose>("/localizer/kinect/add_robot", 1);
	  nostop_agent::Id_robot l_msg;
	  l_msg.name = name_;
	  l_pub.publish(l_msg);
	}