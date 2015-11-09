////////////////////////////////////////////////////
//	iAgent.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_AGENT_H
#define I_AGENT_H
#pragma once

#include "agent.h"

#include "Threads.h"

#include "Configuration.h"
#include "iLocalizer.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>

#include <memory>
#include <string>

namespace Robotics
{
	namespace GameTheory
	{	  
		/// Tutto ciò che serve per la guida del robot!
		class iAgent
	  	{
		protected:
			Configuration m_currentConfiguration;
			Configuration m_targetConfiguration;
			
			std::shared_ptr<iLocalizer> m_localizer;
			
			std::shared_ptr<Agent> m_LAgent;
			
			std::string m_name;
			
			mutable Mutex m_mutex;
			
			ros::NodeHandle m_node;
			ros::ServiceClient m_notifyStatus;
			
			ros::Publisher m_pubMotorControl;
			ros::Subscriber m_subForward;
			ros::Subscriber m_subTargetPoint;
			ros::Subscriber m_subLocalizer;
			ros::Subscriber m_subLaserScan;
			sensor_msgs::LaserScan m_scan;
			
			double m_error_lin_cumulative;
			double m_error_ang_cumulative;
			
			tf::TransformBroadcaster m_broadcaster;
		public:
			int m_motor_control_direction;
		public:
		  		
			void computeConfigurationToTarget();
		  
		  	void updateTargetConfiguration(geometry_msgs::Point const& newTarget_);
			
			void setCurrentConfiguration( Configuration & config_ );
			
			geometry_msgs::Twist getCurrentConfigurationTwist();
			
			geometry_msgs::Point getCurrentConfigurationPosition();
			
			geometry_msgs::Pose getCurrentConfigurationPose();
			//////////////////////////////
		  
			void updateCurrentOdometry( nav_msgs::Odometry & odometry_ );
		  
			void updateCurrentPose( geometry_msgs::Pose & pose_ );
			
			void updateCurrentTwist( geometry_msgs::Twist & twist_ );
			
			void updateCurrentOrientation( geometry_msgs::Quaternion & orientation_);
			
			void updateCurrentPosition( geometry_msgs::Point & currentPosition_);
			
			bool isReal();
			
			geometry_msgs::Point getTargetPoint();
			
			bool isTargetUpdated();
			
			void MoveToNextPosition_LearningAgent();
			
		public:
			iAgent() : m_localizer(nullptr), m_LAgent(nullptr), m_motor_control_direction(-3), m_error_lin_cumulative(0), m_error_ang_cumulative(0) {};
			
			~iAgent() {};
		
			/// Create the link between this agent and the agent of the learning
			void setAgentPtr(std::shared_ptr<Agent> agent_);						
			/// Create a kinect localizer
			void setKinectLocalizer();
			
			/// Create a simulator localizer
			void setSimulatorLocalizer();
			
			Configuration getCurrentConfiguration() {return m_currentConfiguration;}
			Configuration getTargetConfiguration() {return m_targetConfiguration;}
			
			/// Get the robot name
			std::string getName() {return m_name;}
			
			bool isArrived( double tolerance = Math::TOLERANCE);
			
			void setStandByStatus();
			
			void setActiveStatus();
			
			virtual void setName(std::string const& name_);
			
			std::shared_ptr<iLocalizer> getLocalizer();
			
			int getID();
			
			Agent::Status getStatus();
			
			void setTargetConfiguration(Configuration const& target_);
			
		protected:
			  void goForward();
			  void goBackward();
			  void rotateLeft();
			  void rotateRight();
			  void stop();
			  
		public:
			  void updateTargetConfiguration_callback( const std_msgs::Bool::ConstPtr msg_);
			  void computeConfigurationToTarget_callback( const geometry_msgs::Pose::ConstPtr msg_);
			  void updateLaserScan_callback(const sensor_msgs::LaserScan::ConstPtr msg_);
			  void updateTargetPoint_callback( const geometry_msgs::Point::ConstPtr msg_);
	
		protected:
		  
			  void updateTargetPoint( geometry_msgs::Point const& point_);
			  void notifyPositionToTF(const geometry_msgs::Pose & pose_);
			  void computeConfigurationToPoint(const geometry_msgs::Pose & pose_, Real2D const& point_);
			  bool notifyStatus();
			  bool isGoodDirection( double orientation, double tolerance, double min_range);
		};
		
		typedef std::shared_ptr<iAgent> iAgentPtr;
	}
}


#endif // I_AGENT_H