#include "iLocalizer.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	Configuration iLocalizer::getConfiguration() const
	{
		Lock lock(m_mutex);
		m_alreadyRead = true;
		return m_config;
	}

	////////////////////////////////////////////////////
	geometry_msgs::Point iLocalizer::getPosition() const
	{
	  Lock lock(m_mutex);
	  m_alreadyRead = true;
	  return m_config.getPosition();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Quaternion iLocalizer::getOrientation() const
	{
	  Lock lock(m_mutex);
	  m_alreadyRead = true;
	  return m_config.getOrientation();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Twist iLocalizer::getTwist() const
	{
	  Lock lock(m_mutex);
	  m_alreadyRead = true;
	  return m_config.getTwist();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Pose iLocalizer::getPose() const
	{
	  Lock lock(m_mutex);
	  m_alreadyRead = true;
	  return m_config.getPose();
	}
	
	////////////////////////////////////////////////////
	nav_msgs::Odometry iLocalizer::getOdometry() const
	{
	  Lock lock(m_mutex);
	  m_alreadyRead = true;
	  return m_config.getOdometry();
	}
	
	////////////////////////////////////////////////////
	void iLocalizer::updatePose(const geometry_msgs::Pose::ConstPtr & pose_)
	{
	  geometry_msgs::Pose l_pose;
	  l_pose.orientation = pose_->orientation;
	  l_pose.position = pose_->position;

	  Lock l_lock(m_mutex);
	  m_alreadyRead = false;
	  
 	  m_config.setPose(l_pose);
	  
	  m_ready = true;
	}
	
	////////////////////////////////////////////////////
	void iLocalizer::subscribeTopic()
	{
	  m_sub = m_node.subscribe<geometry_msgs::Pose>(m_name.c_str(), 10, &iLocalizer::updatePose, this);
	}
	
	////////////////////////////////////////////////////
	bool iLocalizer::isLocalizerInitialized() const
	{
	  Lock l_lock(m_mutex);
	  return m_ready;
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////
	SimulatorLocalizer::SimulatorLocalizer(std::string name_)
	: iLocalizer(name_)
	{
	  std::string l_agentname = "SimulatorLocalizer_";
	  l_agentname+=m_name;
	  m_name = l_agentname;
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////
	KinectLocalizer::KinectLocalizer(std::string name_) 
	: iLocalizer(name_)
	{
	  std::string l_agentname = "KinectLocalizer_";
	  l_agentname+=m_name;
	  m_name = l_agentname;	  
	}