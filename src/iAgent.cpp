#include "iAgent.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "iLocalizer.h"

#include "Math.h"
#include <math.h>       /* atan2 */

#include "Conversions.h"
#include "Configuration.h"

// #include <tf/tf.h>
#include <boost/pointer_cast.hpp>

#include "GlobalPreProcessor.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
	
	////////////////////////////////////////////////////
	void iAgent::setAgentPtr(std::shared_ptr<Agent> agent_)
	{
	  Lock lock(m_mutex);
	  m_LAgent = agent_;
	}
	
	////////////////////////////////////////////////////
	bool iAgent::setKinectLocalizer()
	try
	{ 
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<KinectLocalizer>(m_name);
	  m_localizer->subscribeTopic();
	  return true;
	}
	catch(...)
	{
	  return false;
	}
	
	////////////////////////////////////////////////////
	bool iAgent::setSimulatorLocalizer()
	try
	{ 
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<SimulatorLocalizer>(m_name);
	  m_localizer->subscribeTopic();
	  return true;
	}
	catch(...)
	{
	  return false;
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
	bool iAgent::isArrived(double tolerance)
	{
	  return m_currentConfiguration.equals(m_targetConfiguration, tolerance);
	  
	  Real2D l_current = Conversions::Point2Real2D( m_currentConfiguration.getPosition() );
	  Real2D l_target = Conversions::Point2Real2D( m_targetConfiguration.getPosition() );
	  
	  Real2D l_delta = l_target-l_current;
	  
	  if( l_delta.mod() < tolerance )
	  {
	    return true;
	  }
	  
	  return false;
	  	  
	  //return m_currentConfiguration.equals(m_targetConfiguration);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setStandByStatus()
	{
	  m_LAgent->setStatus(Agent::STANDBY);
	  this->notifyStatus();
	}
	
	////////////////////////////////////////////////////
	void iAgent::setActiveStatus()
	{
	  m_LAgent->setStatus(Agent::ACTIVE);
	  this->notifyStatus();
	}
		
	////////////////////////////////////////////////////
	double diff_azi(double first, double second)
	{
	  double delta = first - second;
	  while (delta < -Math::Pi) delta += Math::Pi; 
	  while (delta > Math::TwoPi) delta -= Math::TwoPi;
	  return delta;
	}
	
	////////////////////////////////////////////////////
	double ErrorAngle(geometry_msgs::Pose const& cur, Real2D ref)
	{
// 	    tf::Quaternion q(cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w);    
// 	    tf::Matrix3x3 m(q);
// 	    double lroll, lpitch, lyaw;
// 	    m.getRPY(lroll, lpitch, lyaw);

	    double phi   = atan2(2*cur.orientation.y*cur.orientation.x + 2*cur.orientation.w*cur.orientation.z, 
				 1 - 2*cur.orientation.y*cur.orientation.y - 2*cur.orientation.z*cur.orientation.z);
	    
	    double Ex = ref[0] - cur.position.x;   	//errore lungo x
	    double Ey = ref[1] - cur.position.y;   	//errore lungo y  
	    double ref_theta = atan2(Ey, Ex);   	//stima dell'angolo desiderato
	    double Et = ref_theta-phi;   	//errore su theta
	    //ROS_INFO("Angolo Phi: %f\nAngolo ref: %f\n %f, %f\n", phi, ref_theta, Et, ref_theta-phi);
	    return Et;
	}

	////////////////////////////////////////////////////
	double ErrorLinear(geometry_msgs::Pose const& cur, Real2D ref)
	{
	    double Ex = ref[0] - cur.position.x; //errore lungo x
	    double Ey = ref[1] - cur.position.y;           //errore lungo y
	    double Etx = pow(pow(Ex,2)+pow(Ey,2),0.5);
	    return Etx;
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateTargetConfiguration_callback( const std_msgs::Bool::ConstPtr msg_)
	// Update target configuration:
	{
	  geometry_msgs::Point l_tgt_point = this->getTargetPoint();
	  this->updateTargetPoint(l_tgt_point);
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateTargetPoint_callback( const geometry_msgs::Point::ConstPtr msg_)
	{
	  geometry_msgs::Point l_point;
	  l_point.x = msg_->x;
	  l_point.y = msg_->y;
	  l_point.z = msg_->z;
	  this->updateTargetPoint(l_point);
	}
	  
	////////////////////////////////////////////////////
	void iAgent::updateTargetPoint( geometry_msgs::Point const& point_)
	{
	  Lock lock(m_mutex);
	  
	  this->setActiveStatus();
	  this->updateTargetConfiguration( point_ );
	  this->m_motor_control_direction = -3;
	}
	
	////////////////////////////////////////////////////
	void iAgent::notifyPositionToTF(const geometry_msgs::Pose & pose_)
	{
// 	  /// Broadcast new position
// 	  tf::Transform l_transform;
// 	  l_transform.setOrigin( tf::Vector3(pose_.position.x, pose_.position.y, pose_.position.z) );
// 	  
// 	  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
// 	  tf::Quaternion l_orientation;
// 	  tf::quaternionMsgToTF(pose_.orientation, l_orientation);
// 	  l_transform.setRotation( l_orientation );
// 	  m_broadcaster.sendTransform( tf::StampedTransform( l_transform, ros::Time::now(), "world", m_name.c_str() ) );
	}
	
	const double MAX_TWIST_LINEAR = 0.3;
	const double MAX_TWIST_ANGULAR = 2;
	const double CONE_FREE_FOV = .1;
	const int MAX_TRIP_COUNTER = 500;
	
	////////////////////////////////////////////////////
	void iAgent::computeConfigurationToPoint(const geometry_msgs::Pose & pose_, Real2D const& point_)
	{
	  m_counter++;
	  double l_arrived_tolerance = m_motor_control_direction  != -1 ? .3*m_square_side : .5*m_square_side;
	  if( this->isArrived(l_arrived_tolerance) ||  m_counter > MAX_TRIP_COUNTER)
	  {
	      m_error_ang_cumulative = 0;
	      m_error_lin_cumulative = 0;
	      if (m_motor_control_direction  != -1)
		  ROS_INFO("Agent %s is stopping on %.2f, %.2f!\n", m_name.c_str(), pose_.position.x, pose_.position.y);
	      
	      this->stop();
	      m_motor_control_direction  = -1;
	      m_counter = 0;
	      return;
	  }
	  
	  geometry_msgs::Twist l_twist;
	  
	  const double kp1 = .5;
	  const double ki1 = .01;
	  const double kp2 = -1.;
	  const double ki2 = .01;
	  
	  double error_lin = ErrorLinear(pose_, point_);
	  double error_ang = ErrorAngle(pose_, point_);
	  	  	  
	  if(fabs(error_lin) < .5*m_square_side && fabs(error_ang) > .1)
	    l_twist.linear.x = 0;
	  else
	    l_twist.linear.x = kp1*error_lin + ki1 * m_error_lin_cumulative - kp1 * sin(error_ang);
	  
	  l_twist.linear.y = 0;
	  l_twist.linear.z = 0;
	  
	  l_twist.angular.x = 0;
	  l_twist.angular.y = 0;
	  l_twist.angular.z = kp2*sin(error_ang) + ki2 * sin(m_error_ang_cumulative);
	  
	  m_error_lin_cumulative += error_lin;
	  m_error_ang_cumulative += error_ang;
	  
	  // Saturazione sul twist comandato
	  if(fabs(l_twist.linear.x) > MAX_TWIST_LINEAR)
	  {
	    if(m_motor_control_direction != -2)
	      ROS_INFO("Agent %s, to much linear speed!\n", m_name.c_str());
	    l_twist.linear.x = MAX_TWIST_LINEAR * (l_twist.linear.x>0?1.:-1.);
	  }
	      
	  if(fabs(l_twist.angular.z) > MAX_TWIST_ANGULAR)
	  {
	    if(m_motor_control_direction != -2)
	      ROS_INFO("Agent %s, to much angular speed!\n", m_name.c_str());
	    l_twist.angular.z = MAX_TWIST_ANGULAR* (l_twist.angular.z>0?1.:-1.);
	  }
	  
	  this->checkCollisonAvoidance(l_twist, CONE_FREE_FOV);
	  
	  if(m_motor_control_direction != -2)
	    ROS_INFO("Agent %s is going to %.2f, %.2f\n", m_name.c_str(), point_[0], point_[1]);
	  m_motor_control_direction=-2;
	  
	  m_currentConfiguration.setTwist(l_twist);
	  m_pubMotorControl.publish(l_twist);
	}
	
	////////////////////////////////////////////////////
	double minimum(double first, double second)
	{
	  if (first< second)
	    return first;
	  else
	    return second;
	}
	
	////////////////////////////////////////////////////
	void iAgent::checkCollisonAvoidance(geometry_msgs::Twist & twist_, double tolerance)
	{
	  Lock lock(m_mutex);
	  
	  const double k = 1.;
	  
	  if ( !(m_scan.header.seq > 0) )
	    return ;
	  
	  size_t l_gsp_laser_beam_count = m_scan.ranges.size();
	  int l_tolerance_index = floor( (tolerance / m_scan.angle_increment) + .5);
	  
	  double l_range_angular = m_scan.range_max;
	  double l_range_linear = m_scan.range_max;
	  int sign = 0;
	  for(int i = -l_tolerance_index; i <= l_tolerance_index; ++i)
	  {
	    int l_index = (l_gsp_laser_beam_count+i) % l_gsp_laser_beam_count;
	    
	    if ( m_scan.ranges[ l_index ] < minimum(m_square_side,m_scan.range_max) && m_scan.ranges[ l_index ] < l_range_linear)
	    {
	      l_range_linear = m_scan.ranges[ l_index ];
	    }
	    
	    if( m_scan.ranges[ l_index ] < minimum(m_square_side/2.,m_scan.range_max) && m_scan.ranges[ l_index ] < l_range_angular)
	    {
	      l_range_angular = m_scan.ranges[ l_index ];
	      sign = i;
	    }
	  }
	  
	  double l_correction_angular = ( (sign>=0?1:-1) * (m_scan.range_max - l_range_angular) ) * MAX_TWIST_ANGULAR * k;
	  twist_.angular.z += l_correction_angular;
	  
	  double l_correction_linear = (Math::sign(twist_.linear.x)>0?-1:1) * fabs(twist_.linear.x) * (m_scan.range_max - l_range_linear) / m_scan.range_max;
	  twist_.linear.x += l_correction_linear;
	  
	  //ROS_INFO("Correction Linear%.3f!\n", l_correction_linear );
	  //ROS_INFO("Correction Angular %.3f!\n", l_correction_angular );
	}
	
	////////////////////////////////////////////////////
	void iAgent::computeConfigurationToTarget_callback( const geometry_msgs::Pose::ConstPtr msg_)
	{
	  Lock lock(m_mutex);
	  geometry_msgs::Pose l_pose;
	  l_pose.orientation = msg_->orientation;
	  l_pose.position = msg_->position;
	  
	  this->updateCurrentPosition( l_pose.position );
	  this->updateCurrentOrientation( l_pose.orientation );
	  
	  this->notifyPositionToTF(l_pose);
	  
	  Real2D l_target = Conversions::Point2Real2D( m_targetConfiguration.getPosition() );
	  return this->computeConfigurationToPoint(l_pose, l_target);
	}
	
	////////////////////////////////////////////////////
	bool iAgent::isGoodDirection( double orientation_, double tolerance, double min_range)
	{
	  Lock lock(m_mutex);
	  
	  if ( !(m_scan.header.seq > 0) )
	    return true;
	  
	  size_t l_gsp_laser_beam_count = m_scan.ranges.size();
	  int l_tolerance_index = floor( (tolerance / m_scan.angle_increment) + .5);
	  
	  for(int i = -l_tolerance_index; i <= l_tolerance_index; ++i)
	  {
	    int l_index = (l_gsp_laser_beam_count+i) % l_gsp_laser_beam_count;
	    double l_range = m_scan.ranges[ l_index ];
	    if (l_range < min_range)
	      return false;
	  }
	  
	  return true;
	}
	
	////////////////////////////////////////////////////
	void iAgent::updateLaserScan_callback(const sensor_msgs::LaserScan::ConstPtr msg_)
	{
	  Lock lock(m_mutex);
	  m_scan.angle_increment = msg_->angle_increment;
	  m_scan.angle_max = msg_->angle_max;
	  m_scan.angle_min = msg_->angle_min;
	  m_scan.header = msg_->header;
	  m_scan.intensities = msg_->intensities;
	  m_scan.range_max = msg_->range_max;
	  m_scan.range_min = msg_->range_min;
	  m_scan.ranges = msg_->ranges;
	  m_scan.scan_time = msg_->scan_time;
	  m_scan.time_increment = msg_->time_increment;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setName(std::string const& name_)
	{
	    Lock lock(m_mutex);
	    m_name = name_;
	    
	    std::string l_name = "/";
	    l_name += m_name;
	    
	    std::string l_cmd_vel = l_name;
	    l_cmd_vel += "/cmd_vel";
#ifdef _DEBUG_PRINT
	    std::cout << "*** Advertise iAgent "<< l_cmd_vel <<std::endl;
#endif
	    m_pubMotorControl = m_node.advertise<geometry_msgs::Twist>(l_cmd_vel.c_str(), 1);
	    
	    std::string l_learning_name = l_name;
	    l_learning_name += "/update";
#ifdef _DEBUG_PRINT	    
	    std::cout << "Subscribe iAgent "<< l_learning_name <<std::endl;
#endif
	    m_subForward = m_node.subscribe<std_msgs::Bool::ConstPtr>(l_learning_name.c_str(), 1, &iAgent::updateTargetConfiguration_callback, this);
	    
	    std::string l_point_name = l_name;
	    l_point_name += "/update/point";
#ifdef _DEBUG_PRINT	    
	    std::cout << "Subscribe iAgent "<< l_point_name <<std::endl;
#endif
	    m_subTargetPoint = m_node.subscribe<geometry_msgs::Point::ConstPtr>(l_point_name.c_str(), 1, &iAgent::updateTargetPoint_callback, this);
	    
	    std::string l_localizer_name = l_name;
	    l_localizer_name += "/localizer/pose";
#ifdef _DEBUG_PRINT 
	    std::cout << "Subscribe iAgent "<< l_localizer_name <<std::endl;
#endif
	    m_subLocalizer = m_node.subscribe<geometry_msgs::Pose::ConstPtr>(l_localizer_name.c_str(), 1, &iAgent::computeConfigurationToTarget_callback, this);
	    
	    std::string l_laser_scan_name = l_name;
	    l_laser_scan_name += "/laser_scan";
#ifdef _DEBUG_PRINT
	    std::cout << "Subscribe iAgent "<< l_laser_scan_name <<std::endl;
#endif
	    m_subLaserScan = m_node.subscribe<sensor_msgs::LaserScan::ConstPtr>(l_laser_scan_name.c_str(), 1, &iAgent::updateLaserScan_callback, this);
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
	void iAgent::updateTargetConfiguration(geometry_msgs::Point const& newTarget_)
	{
	  Lock lock(m_mutex);
	  m_targetConfiguration.setPosition(newTarget_);
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
	  m_pubMotorControl.publish(l_twist);
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
	  m_pubMotorControl.publish(l_twist);
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
	  m_pubMotorControl.publish(l_twist);
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
	  m_pubMotorControl.publish(l_twist);
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
	  m_pubMotorControl.publish(l_twist);
	}
	
	////////////////////////////////////////////////////
	bool iAgent::isReal()
	{
	  return m_localizer->isReal();
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
	
	////////////////////////////////////////////////////	
	void iAgent::setTargetConfiguration(Configuration const& target_)
	{
	  Lock lock(m_mutex);
	  m_targetConfiguration = target_;
	}