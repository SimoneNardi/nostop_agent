#include "Configuration.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	bool Configuration::equals( const Configuration& rhs, double tolerance ) const
	{
	  bool l_equals;
// 	  geometry_msgs::Quaternion lhs_orientation = lhs.getOrientation();
	  geometry_msgs::Point lhs_position = this->getPosition();
	  
// 	  geometry_msgs::Quaternion rhs_orientation = rhs.getOrientation();
	  geometry_msgs::Point rhs_position = rhs.getPosition();
	  
	  l_equals = fabs(lhs_position.x - rhs_position.x) < tolerance /*Math::TOLERANCE*/ && 
		 fabs(lhs_position.y - rhs_position.y) < tolerance /*Math::TOLERANCE*/ && 
		 fabs(lhs_position.z - rhs_position.z) < tolerance /*Math::TOLERANCE*/;
	  
	  return l_equals;
	}

	////////////////////////////////////////////////////
	Configuration::Configuration ()
	{}
	
	////////////////////////////////////////////////////
	Configuration::~Configuration()
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
	void Configuration::setPosition(const geometry_msgs::Point & position_)
	{
	  m_odom.pose.pose.position = position_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setOrientation(const geometry_msgs::Quaternion & orientation_)
	{
	  m_odom.pose.pose.orientation = orientation_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setPose(const geometry_msgs::Pose & pose_)
	{
	  m_odom.pose.pose = pose_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setTwist(const geometry_msgs::Twist & twist_)
	{
	  m_odom.twist.twist = twist_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setOdometry(const nav_msgs::Odometry & odometry_)
	{
	  m_odom = odometry_;
	}
	
	////////////////////////////////////////////////////
	void Configuration::setConfiguration(const Configuration & config_)
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