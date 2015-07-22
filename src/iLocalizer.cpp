#include "iLocalizer.h"

#include <string>

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	Configuration iLocalizer::getConfiguration() const
	{
		Lock lock(m_mutex);
		return m_config;
	}

	////////////////////////////////////////////////////
	geometry_msgs::Point iLocalizer::getPosition() const
	{
	  Lock lock(m_mutex);
	  return m_config.getPosition();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Quaternion iLocalizer::getOrientation() const
	{
	  Lock lock(m_mutex);
	  return m_config.getOrientation();
	}
	
	geometry_msgs::Twist iLocalizer::getTwist() const
	{
	  Lock lock(m_mutex);
	  return m_config.getTwist();
	}
	
	geometry_msgs::Pose iLocalizer::getPose() const
	{
	  Lock lock(m_mutex);
	  return m_config.getPose();
	}
	
	nav_msgs::Odometry iLocalizer::getOdometry() const
	{
	  Lock lock(m_mutex);
	  return m_config.getOdometry();
	  
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////
	SimulatorLocalizer::SimulatorLocalizer(std::string name_)
	: iLocalizer(name_)
	{}
	
	////////////////////////////////////////////////////
	void SimulatorLocalizer::subscribe()
	{
	  std::string l_agentname = "SimulatorLocalizer_";
	  l_agentname+=m_name;
	  m_sub = m_node.subscribe(l_agentname.c_str(), 10, &SimulatorLocalizer::update, this);
	}
	
	////////////////////////////////////////////////////
	void SimulatorLocalizer::update(geometry_msgs::PoseConstPtr & pose_)
	{
	  Lock l_lock(m_mutex);
	  
	  // TODO: chiede al simulotre la configurazione del robot
	}
	
	////////////////////////////////////////////////////
	void SimulatorLocalizer::updatePosition()
	{
	  // TODO: chiede al simulotre la posizione del robot
	}
	
	////////////////////////////////////////////////////
	void SimulatorLocalizer::updateOrientation()
	{
	  // TODO: chiede al simulotre l'orientazione del robot
	}
							

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////
	KinectLocalizer::KinectLocalizer(std::string name_) 
	: iLocalizer(name_)
	{}
	
	////////////////////////////////////////////////////
	void KinectLocalizer::subscribe()
	{
	  std::string l_agentname = "KinectLocalizer_";
	  l_agentname+=m_name;
	  m_sub = m_node.subscribe(l_agentname.c_str(), 10, &KinectLocalizer::update, this);
	}
	
	////////////////////////////////////////////////////
	void KinectLocalizer::update(geometry_msgs::PoseConstPtr & pose_)
	{
	  Lock l_lock(m_mutex);
	  
	  // TODO: chiede al kinect la configurazione del robot
	}
	
	////////////////////////////////////////////////////
	void KinectLocalizer::updatePosition()
	{
	  // TODO: chiede al kinect la posizione del robot
	}
	
	////////////////////////////////////////////////////
	void KinectLocalizer::updateOrientation()
	{
	  // TODO: chiede al kinect l'orientazione del robot
	}