#include "iAgent.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	bool operator==(const Configuration& lhs, const Configuration& rhs)
	{
// 	  geometry_msgs::Quaternion lhs_orientation = lhs.getOrientation();
	  geometry_msgs::Point lhs_position = lhs.getPosition();
	  
// 	  geometry_msgs::Quaternion rhs_orientation = rhs.getOrientation();
	  geometry_msgs::Point rhs_position = rhs.getPosition();
	  
	  return lhs_position==rhs_position;
	}

	////////////////////////////////////////////////////
	bool operator!=(const Configuration& lhs, const Configuration& rhs)
	{
	  return !(lhs==rhs);
	}
	
	Configuration::Configuration (nav_msgs::OdometryConstPtr& odom_)
	{
	  m_odom = odom_.get();
	}
	
	////////////////////////////////////////////////////
	Configuration::Configuration (geometry_msgs::PoseConstPtr & pose_)
	{
	  m_odom.pose.pose = pose_.get();
	}
	
	////////////////////////////////////////////////////
	void Configuration::setPosition(geometry_msgs::Point & position_)
	{
	  m_odom.pose.pose.position = position_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setOrientation(geometry_msgs::Quaternion & orientation_)
	{
	  m_odom.pose.pose.orientation = orientation_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setPose(geometry_msgs::Pose & pose_)
	{
	  m_odom.pose.pose = pose_;
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Point Configuration::getPosition()
	{
	  return m_odom.pose.pose.position;
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Quaternion Configuration::getOrientation()
	{
	  return m_odom.pose.pose.orientation;
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Pose Configuration::getPose()
	{
	  return m_odom.pose.pose;
	}

	////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
	
	////////////////////////////////////////////////////
	void iAgent::setCurrentOrientation(geometry_msgs::Quaternion & orientation_) 
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setOrientation(orientation_);
	}
	
	////////////////////////////////////////////////////
	void setCurrentPosition(geometry_msgs::Point & point_)
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setPosition(point_);
	}

	////////////////////////////////////////////////////
	AgentPosition getCurrentPosition()
	{
	  //TODO ... calcolo dell'agentPosition.
	  AgentPosition l_pos;
	  return l_pos;
	}
	
	////////////////////////////////////////////////////
	AgentPosition  iAgent::computeAgentPosition(geometry_msgs::Point & point_)
	{
	  Lock lock(m_mutex);
	  AgentPosition l_pos;
	  return l_pos;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setAgentPtr(std::shared_ptr<Agent> agent_)
	{
	  Lock lock(m_mutex);
	  m_agent = agent_;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setLocalizer(std::string name_, ColorName back_, ColorName front_)
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<KinectLocalizer>(name_, back_, front_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setLocalizer(std::string name_)
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<SimulatorLocalizer>(name_);
	}
	
	////////////////////////////////////////////////////
	double iAgent::getCurrentAngularSpeed()
	{
	  // TODO
	  
	  return 0.;
	}
	
	////////////////////////////////////////////////////
	double iAgent::getCurrentLinearSpeed()
	{
	  // TODO

	  return 0.;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setCurrentConfiguration( geometry_msgs::PoseConstPtr & pose_ )
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration = Configuration(pose_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setCurrentConfiguration( Configuration & config_ )
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration = config_;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setCurrentConfiguration( nav_msgs::OdometryConstPtr & odometry_ )
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration = Configuration(odometry_);
	}
	
	////////////////////////////////////////////////////
	bool iAgent::isArrived()
	{
	  return m_currentConfiguration == m_targetConfiguration;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setStandByStatus()
	{
	  m_agent->setStatus(Agent::STANDBY);
	}
			
	////////////////////////////////////////////////////
	void iAgent::setActiveStatus()
	{
	  m_agent->setStatus(Agent::ACTIVE);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setName(std::string name_)
	{
	    m_name = name_;
	}
	