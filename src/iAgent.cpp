#include "iAgent.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"

#include "iLocalizer.h"

#include "IDSMath.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	bool Configuration::equals( const Configuration& rhs ) const
	{
// 	  geometry_msgs::Quaternion lhs_orientation = lhs.getOrientation();
	  geometry_msgs::Point lhs_position = this->getPosition();
	  
// 	  geometry_msgs::Quaternion rhs_orientation = rhs.getOrientation();
	  geometry_msgs::Point rhs_position = rhs.getPosition();
	  
	  return fabs(lhs_position.x - rhs_position.x) < IDSMath::TOLERANCE && 
		 fabs(lhs_position.y - rhs_position.y) < IDSMath::TOLERANCE && 
		 fabs(lhs_position.z - rhs_position.z) < IDSMath::TOLERANCE;
	}

	Configuration::Configuration (nav_msgs::Odometry& odom_)
	{
	  m_odom = odom_;
	}
	
	////////////////////////////////////////////////////
	Configuration::Configuration (geometry_msgs::Pose & pose_)
	{
	  m_odom.pose.pose = pose_;
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
	geometry_msgs::Point Configuration::getPosition() const
	{
	  return m_odom.pose.pose.position;
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Quaternion Configuration::getOrientation() const
	{
	  return m_odom.pose.pose.orientation;
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Pose Configuration::getPose() const
	{
	  return m_odom.pose.pose;
	}

	////////////////////////////////////////////////////
	geometry_msgs::Twist Configuration::getTwist() const
	{
	  return m_odom.twist.twist;
	}
		  
	////////////////////////////////////////////////////
	nav_msgs::Odometry Configuration::getOdometry() const
	{
	  return m_odom;
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
	void iAgent::setCurrentPosition(geometry_msgs::Point & point_)
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setPosition(point_);
	}

	////////////////////////////////////////////////////
	void iAgent::setAgentPtr(std::shared_ptr<Agent> agent_)
	{
	  Lock lock(m_mutex);
	  m_LAgent = agent_;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setKinectLocalizer(std::string name_)
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<KinectLocalizer>(name_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setSimulatorLocalizer(std::string name_)
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
	void iAgent::setCurrentConfiguration( geometry_msgs::Pose & pose_ )
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
	void iAgent::setCurrentConfiguration( nav_msgs::Odometry & odometry_ )
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration = Configuration(odometry_);
	}
	
	////////////////////////////////////////////////////
	bool iAgent::isArrived()
	{
	  return m_currentConfiguration.equals(m_targetConfiguration);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setStandByStatus()
	{
	  m_LAgent->setStatus(Agent::STANDBY);
	}
			
	////////////////////////////////////////////////////
	void iAgent::setActiveStatus()
	{
	  m_LAgent->setStatus(Agent::ACTIVE);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setName(std::string name_)
	{
	    m_name = name_;
	}
	