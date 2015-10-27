#include "iAgent.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"

#include "nostop_agent/PlayerNotifyStatus.h"

#include "iLocalizer.h"

#include "Math.h"

#include "Conversions.h"

#include <tf/tf.h>

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	bool Configuration::equals( const Configuration& rhs ) const
	{
	  bool l_equals;
// 	  geometry_msgs::Quaternion lhs_orientation = lhs.getOrientation();
	  geometry_msgs::Point lhs_position = this->getPosition();
	  
// 	  geometry_msgs::Quaternion rhs_orientation = rhs.getOrientation();
	  geometry_msgs::Point rhs_position = rhs.getPosition();
	  
	  l_equals = fabs(lhs_position.x - rhs_position.x) < 0.3 /*Math::TOLERANCE*/ && 
		 fabs(lhs_position.y - rhs_position.y) < 0.3 /*Math::TOLERANCE*/ && 
		 fabs(lhs_position.z - rhs_position.z) < 0.3 /*Math::TOLERANCE*/;
	  
	  return l_equals;
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
	  
	  std::string  l_name = "/publisher/status";
  	  m_notifyStatus = m_node.serviceClient<nostop_agent::PlayerNotifyStatus>(l_name.c_str());
	}
	
	////////////////////////////////////////////////////
	void iAgent::setKinectLocalizer()
	{ 
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<KinectLocalizer>(m_name);
	  m_localizer->subscribeTopic();
	}
	
	////////////////////////////////////////////////////
	void iAgent::setSimulatorLocalizer()
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<SimulatorLocalizer>(m_name);
	  m_localizer->subscribeTopic();
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
	  l_srv.request.status = this->getStatus();
	  
	  if(!m_notifyStatus.exists())
	    m_notifyStatus.waitForExistence();
	  	  
	  if ( !m_notifyStatus.exists() || !m_notifyStatus.call(l_srv) )
	  {
	      ROS_ERROR("Failed to call service Notify Status");
	      return false;
	  }
	  else
	  {
	      ROS_INFO("Notify Status of Agent %d: Status %d", l_srv.request.id, l_srv.request.status);
	  }

	  return true;
	}
			
	////////////////////////////////////////////////////
	void iAgent::setActiveStatus()
	{
	  m_LAgent->setStatus(Agent::ACTIVE);
	  this->notifyStatus();
	}
	
	////////////////////////////////////////////////////
	void iAgent::setName(std::string const& name_)
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
	
	const double MAX_TWIST_LINEAR = 0.5;
	const double MAX_TWIST_ANGULAR = 0.5;
	
	////////////////////////////////////////////////////
	double ErrorAngle(nav_msgs::Odometry cur, Real2D ref)
	{
	    tf::Quaternion q(cur.pose.pose.orientation.x, cur.pose.pose.orientation.y, cur.pose.pose.orientation.z, cur.pose.pose.orientation.w);    
	    tf::Matrix3x3 m(q);
	    double roll, pitch, yaw;
	    m.getRPY(roll, pitch, yaw);
	      
	    double Ex = ref[0] - cur.pose.pose.position.x;   //errore lungo x
	    double Ey = ref[1] - cur.pose.pose.position.y;   //errore lungo y  
	    double ref_theta = atan2(Ey, Ex);   //stima dell'angolo desiderato
	    double cur_theta = yaw;
	    double Et = ref_theta-cur_theta;   //errore su theta
	    return Et;
	}

	////////////////////////////////////////////////////
	double ErrorLinear(nav_msgs::Odometry cur, Real2D ref)
	{
	    double Ex = ref[0] - cur.pose.pose.position.x;           //errore lungo x
	    double Ey = ref[1] - cur.pose.pose.position.y;           //errore lungo y
	    double Etx = pow(pow(Ex,2)+pow(Ey,2),0.5);
	    return Etx;
	}
		
	////////////////////////////////////////////////////
	void iAgent::computeConfigurationToTarget()
	{
	  Lock lock(m_mutex);
	  Real2D l_current = Conversions::Point2Real2D( m_currentConfiguration.getPosition() );
	  Real2D l_target = Conversions::Point2Real2D( m_targetConfiguration.getPosition() );
	  
	  Real2D l_delta = l_target-l_current;
	  
	  if( l_delta.mod() < 0.3 )
	  {
	    if (m_motor_control_direction  != -1)
	      ROS_INFO("Stop Moving!\nTarget %.2f, %.2f reached!\n", l_target[0], l_target[1]);
	    this->stop();
	    m_motor_control_direction  = -1;
	    return;
	  }
	  
	  double l_phi = Math::polarPhi2D(l_delta);
	  
	  auto l_quat_orient = m_currentConfiguration.getOrientation();
	  
	  double roll  = atan2(2*l_quat_orient.y*l_quat_orient.w - 2*l_quat_orient.x*l_quat_orient.z, 1 - 2*l_quat_orient.y*l_quat_orient.y - 2*l_quat_orient.z*l_quat_orient.z);
	  double pitch = atan2(2*l_quat_orient.x*l_quat_orient.w - 2*l_quat_orient.y*l_quat_orient.z, 1 - 2*l_quat_orient.x*l_quat_orient.x - 2*l_quat_orient.z*l_quat_orient.z);
	  double yaw   = asin(2*l_quat_orient.x*l_quat_orient.y + 2*l_quat_orient.z*l_quat_orient.w);
	  
	  double rho = l_delta.mod();
	  double phi = atan(l_delta[1]/l_delta[0]) + Math::Pi;
	  double alpha = phi - yaw;
	  
	  double k1 = 1., k2 = 1.;
	  double lambda2 = .5;
	  
	  double w = k1 * cos(alpha);
	  double omega = k1 * sin(alpha) / alpha * cos( alpha*alpha + alpha*phi * lambda2 ) + k2 * alpha;
	  	  
	  geometry_msgs::Twist l_twist;
	  
	  double kp1 = .5;
	  double kp2 = .5;
	  
	  nav_msgs::Odometry l_current_odom = m_currentConfiguration.getOdometry();
	  
	  double error_lin = ErrorLinear(l_current_odom, l_target);
	  double error_ang = ErrorAngle(l_current_odom, l_target);
	  
	  l_twist.linear.x = kp1*error_lin; 
	  l_twist.linear.y = 0;
	  l_twist.linear.z = 0;
	  
	  l_twist.angular.x = 0;
	  l_twist.angular.y = 0;
	  l_twist.angular.z = kp2*sin(error_ang);
	  
	  // Saturazione sul twist comandato
	  if(l_twist.linear.x > MAX_TWIST_LINEAR)
	      l_twist.linear.x = MAX_TWIST_LINEAR;
	      
	  if(l_twist.angular.z > MAX_TWIST_ANGULAR)
	      l_twist.angular.z = MAX_TWIST_ANGULAR;
	  
	  if(m_motor_control_direction != -2)
	    ROS_INFO("Go to %.2f, %.2f\n", l_target[0], l_target[1]);
	  m_motor_control_direction=-2;
	  
	  m_currentConfiguration.setTwist(l_twist);
	  return;
	  
	  double l_tolerance = 0.1;//100.*Math::TOLERANCE;
	  if ( fabs(l_phi-yaw) < l_tolerance || fabs(l_phi-yaw-Math::Pi) < l_tolerance)
	    // movimento lineare:
	  {
	    if(fabs(l_phi-yaw) > l_tolerance)
	    {
	      if (m_motor_control_direction != 0)
		ROS_INFO("Go Backward!\nGo to %.2f, %.2f\n", l_target[0], l_target[1]);
	      this->goBackward();
	      m_motor_control_direction  = 0;
	    }
	    else
	    {
	      if (m_motor_control_direction  != 1)
		ROS_INFO("Go Forward!\nGo to %.2f, %.2f\n", l_target[0], l_target[1]);
	      this->goForward();
	      m_motor_control_direction  = 1;
	    }
	  
	  }
	  else
	    // allineamento degli heading:
	  {
	    if (l_phi-yaw>0)
	    {
	      if (m_motor_control_direction  != 2)
		ROS_INFO("Rotate left!\nGo to %.2f, %.2f\n", l_target[0], l_target[1]);
	      this->rotateLeft();
	      m_motor_control_direction  = 2;
	    }
	    else
	    {
	      if (m_motor_control_direction  != 3)
		ROS_INFO("Rotate right!\nGo to %.2f, %.2f\n", l_target[0], l_target[1]);
	      this->rotateRight();
	      m_motor_control_direction  = 3;
	    }
	  }
	  return;
	}
	
	////////////////////////////////////////////////////
	int iAgent::getID()
	{
	  Lock lock(m_mutex);
	  return m_LAgent->getID();
	}
	
	////////////////////////////////////////////////////
	Agent::Status iAgent::getStatus()
	{
	    Lock lock(m_mutex);
	    return m_LAgent->getStatus();
	}

	////////////////////////////////////////////////////
	std::shared_ptr<iLocalizer> iAgent::getLocalizer()
	{
	  return m_localizer;
	}
	
	double g_value_linear = .3;
	double g_value_angular = .1;
	
	////////////////////////////////////////////////////
	void iAgent::goForward()
	{
	  geometry_msgs::Twist l_twist;
	  l_twist.linear.x = g_value_linear;
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
	  l_twist.linear.x = -g_value_linear;
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
	  l_twist.angular.z = g_value_angular;
	  
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
	  l_twist.angular.z = -g_value_angular;
	  
	  m_currentConfiguration.setTwist(l_twist);
	}	
	
	////////////////////////////////////////////////////
	void iAgent::stop()
	{
	  geometry_msgs::Twist l_twist;
	  l_twist.linear.x = 0;
	  l_twist.linear.y = 0;
	  l_twist.linear.z = 0;
	  
	  l_twist.angular.x = 0;
	  l_twist.angular.y = 0;
	  l_twist.angular.z = 0;
	  
	  m_currentConfiguration.setTwist(l_twist);
	}
	
	////////////////////////////////////////////////////
	bool iAgent::isReal()
	{
	  KinectLocalizerPtr l_localizer = std::static_pointer_cast<KinectLocalizer>(m_localizer);
	  return l_localizer == nullptr;
	}
	
	////////////////////////////////////////////////////
	geometry_msgs::Point iAgent::getTargetPoint()
	{
	  AgentPosition l_pos = m_LAgent->getNextPosition();
	  Real2D l_point = l_pos.getPoint2D();
	  geometry_msgs::Point l_res;
	  l_res.x = l_point[0];
	  l_res.y = l_point[1];
	  return l_res;
	}
	
	////////////////////////////////////////////////////
	bool iAgent::isTargetUpdated()
	{
	  return m_LAgent->isTargetUpdated();
	}
	
	////////////////////////////////////////////////////
	void iAgent::MoveToNextPosition_LearningAgent()
	{
	  m_LAgent->moveToNextPosition();
	}