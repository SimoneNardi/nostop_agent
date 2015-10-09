#include "iAgent.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"

#include "nostop_agent/PlayerNotifyStatus.h"

#include "iLocalizer.h"

#include "Math.h"

#include "Conversions.h"

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
	  
	  return fabs(lhs_position.x - rhs_position.x) < Math::TOLERANCE && 
		 fabs(lhs_position.y - rhs_position.y) < Math::TOLERANCE && 
		 fabs(lhs_position.z - rhs_position.z) < Math::TOLERANCE;
	}

	////////////////////////////////////////////////////
	Configuration::Configuration ()
	{}
	
	////////////////////////////////////////////////////
	Configuration::Configuration (nav_msgs::Odometry const& odom_)
	{
	  m_odom = odom_;
	}
	
	////////////////////////////////////////////////////
	Configuration::Configuration (geometry_msgs::Pose const& pose_)
	{
	  m_odom.pose.pose = pose_;
	}
	
	////////////////////////////////////////////////////
	Configuration::Configuration (geometry_msgs::Point const& point_)
	{
	  m_odom.pose.pose.position = point_;
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
	void Configuration::setTwist(geometry_msgs::Twist & twist_)
	{
	  m_odom.twist.twist = twist_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setOdometry(nav_msgs::Odometry & odometry_)
	{
	  m_odom = odometry_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setConfiguration(Configuration & config_)
	{
	  m_odom = config_.getOdometry();
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
	void iAgent::setAgentPtr(std::shared_ptr<Agent> agent_)
	{
	  Lock lock(m_mutex);
	  m_LAgent = agent_;
	  m_notifyStatus = m_node.serviceClient<nostop_agent::PlayerNotifyStatus>("NotifyStatus");
	}
	
	////////////////////////////////////////////////////
	void iAgent::setKinectLocalizer()
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<KinectLocalizer>(m_name);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setSimulatorLocalizer()
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<SimulatorLocalizer>(m_name);
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Twist iAgent::getCurrentConfigurationTwist()
	{
	  Lock lock(m_mutex);
	  return m_currentConfiguration.getTwist();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Point iAgent::getCurrentConfigurationPosition()
	{
	  Lock lock(m_mutex);
	  return m_currentConfiguration.getPosition();
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Pose iAgent::getCurrentConfigurationPose()
	{
	  Lock lock(m_mutex);
	  return m_currentConfiguration.getPose();
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
	  this->notifyStatus();
	}
	
	//////////////////////////////////////////////////
	// send a broadcast message of unemployed agents
	bool iAgent::notifyStatus()
	{
	  nostop_agent::PlayerNotifyStatus l_srv;
	  l_srv.request.id = this->getID();
	  if ( !m_notifyStatus.call(l_srv) )
	  {
	      ROS_ERROR("Failed to call service Notify Status");
	      return false;
	  }

	  return true;
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
	
	////////////////////////////////////////////////////
	void iAgent::updateCurrentOdometry( nav_msgs::Odometry & odometry_ )
	{
	  Lock lock(m_mutex);
		m_currentConfiguration.setOdometry(odometry_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateCurrentPose( geometry_msgs::Pose & pose_ )
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setPose(pose_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateCurrentTwist( geometry_msgs::Twist & twist_ )
	{
	  Lock lock(m_mutex);
	   m_currentConfiguration.setTwist(twist_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateCurrentOrientation(geometry_msgs::Quaternion & orientation_)
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setOrientation(orientation_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateCurrentPosition(geometry_msgs::Point & position_)
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setPosition(position_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setCurrentConfiguration( Configuration & config_ )
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setConfiguration(config_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateTargetConfiguration(geometry_msgs::Point & newTarget_)
	{
	  Lock lock(m_mutex);
	  m_targetConfiguration.setPosition(newTarget_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::computeConfigurationToTarget()
	{
	  Lock lock(m_mutex);
	  
	  Real2D l_current = Conversions::Point2Real2D(m_currentConfiguration.getPosition());
	  Real2D l_target = Conversions::Point2Real2D(m_targetConfiguration.getPosition());
	  
	  Real2D l_delta = l_target-l_current;
	  
	  double l_phi = Math::polarPhi2D(l_delta);
	  
	  double l_tolerance = 100.*Math::TOLERANCE;
	  if (fabs(l_phi) > l_tolerance || fabs(l_phi-Math::Pi) > l_tolerance)
	  // allineamento degli heading:
	  {
	    if (l_phi>0)
	      this->rotateLeft();
	    else
	      this->rotateRight();
	  }
	  else
	  // movimento lineare:
	  {
	    if(fabs(l_phi-Math::Pi) > l_tolerance)
	      this->goBackward();
	    else
	      this->goForward();
	  }
	}
	
	////////////////////////////////////////////////////
	int iAgent::getID()
	{
	  Lock lock(m_mutex);
	  m_LAgent->getID();
	}
	
	////////////////////////////////////////////////////
	Agent::Status iAgent::getStatus()
	{
	    Lock lock(m_mutex);
	    m_LAgent->getStatus();
	}

	////////////////////////////////////////////////////
	std::shared_ptr<iLocalizer> iAgent::getLocalizer()
	{
	  return m_localizer;
	}
	
	////////////////////////////////////////////////////
	void iAgent::goForward()
	{
	  geometry_msgs::Twist l_twist;
	  l_twist.linear.x = 1;
	  l_twist.linear.y = 0;
	  l_twist.linear.z = 0;
	  
	  l_twist.angular.x = 0;
	  l_twist.angular.y = 0;
	  l_twist.angular.z = 0;
	  
	  m_currentConfiguration.setTwist(l_twist);
	}
	
	////////////////////////////////////////////////////
	void iAgent::goBackward()
	{
	  geometry_msgs::Twist l_twist;
	  l_twist.linear.x = -1;
	  l_twist.linear.y = 0;
	  l_twist.linear.z = 0;
	  
	  l_twist.angular.x = 0;
	  l_twist.angular.y = 0;
	  l_twist.angular.z = 0;
	  
	  m_currentConfiguration.setTwist(l_twist);
	}
	
	////////////////////////////////////////////////////
	void iAgent::rotateLeft()
	{
	  geometry_msgs::Twist l_twist;
	  l_twist.linear.x = 0;
	  l_twist.linear.y = 0;
	  l_twist.linear.z = 0;
	  
	  l_twist.angular.x = 0;
	  l_twist.angular.y = 0;
	  l_twist.angular.z = 1;
	  
	  m_currentConfiguration.setTwist(l_twist);
	}
	
	////////////////////////////////////////////////////
	void iAgent::rotateRight()
	{
	  geometry_msgs::Twist l_twist;
	  l_twist.linear.x = 0;
	  l_twist.linear.y = 0;
	  l_twist.linear.z = 0;
	  
	  l_twist.angular.x = 0;
	  l_twist.angular.y = 0;
	  l_twist.angular.z = -1;
	  
	  m_currentConfiguration.setTwist(l_twist);
	}	