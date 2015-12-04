#include "iLocalizer.h"

#include <nostop_agent/AddRobot.h>

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
	
	iLocalizer::iLocalizer(std::string name_) 
	: m_initialized(false)
	, m_updated(false) 
	{
	  std::string l_agentname = "/";
	  l_agentname = "/";
	  l_agentname += name_;
	  l_agentname += "/localizer/pose";
	  m_pub_name = l_agentname; 
	};
	
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
	  l_agentname += "/localizer/camera/pose";
	  m_sub_name = l_agentname;	  
	  
	  // Add robot to sensor localizer
	  ros::ServiceClient l_add_robot_client = m_node.serviceClient<nostop_agent::AddRobot>("/localizer/add_robot");
	  
	  nostop_agent::AddRobot l_msg;
	  l_msg.request.name = name_.c_str();
	  
	  if (l_add_robot_client.call(l_msg))
	  {
	    ROS_INFO("Add Robot %s to camera localizer", name_.c_str());
	  }
	  else
	  {
	    ROS_ERROR("%s: Failed to call service /localizer/add_robot!", name_.c_str());
	  }
	}