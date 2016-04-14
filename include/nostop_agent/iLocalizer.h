////////////////////////////////////////////////////
//	iLocalizer.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_LOCALIZER_H
#define I_LOCALIZER_H
#pragma once

#include "ThreadBase.h"

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#include "ros/ros.h"

#include "Configuration.h"

namespace Robotics
{
	namespace GameTheory
	{
	  	/// Localization sensor for robot.
		class iLocalizer
		{
		protected:
		  
		  Configuration m_config;

		  mutable Mutex m_mutex;
		  
		  std::string m_sub_name;
		  std::string m_pub_name;
		  
		  ros::NodeHandle m_node;
		  ros::Subscriber m_sub;
		  ros::Publisher m_pub;
		  
		  mutable bool m_initialized;
		  mutable bool m_updated;

		public:
			iLocalizer(std::string name_);
			
			~iLocalizer() {};
			
			geometry_msgs::Point getPosition(); 
			
			geometry_msgs::Quaternion getOrientation();
			
			geometry_msgs::Twist getTwist(); 
			
			geometry_msgs::Pose getPose();
			
			nav_msgs::Odometry getOdometry();
			
			Configuration getConfiguration();
			
			void updatePose(const geometry_msgs::Pose::ConstPtr  & pose_);
			
			void subscribeTopic();
			
			bool isInitialized();
			
			bool isUpdated();
			
			virtual bool isReal() = 0;
		};
		
		typedef std::shared_ptr<iLocalizer> iLocalizerPtr;
		
		/// Localization sensor for robot using the kinect.
		class KinectLocalizer: public iLocalizer
	  	{		  
		  	ros::Subscriber m_kinect_sub;
			ros::Publisher m_kinect_pub;
		public:
			KinectLocalizer(std::string name_);
			
			~KinectLocalizer() {};
			
			void updateOdometry(const nav_msgs::Odometry::ConstPtr& odometry_);
			void updatePoseWithCovariance(const geometry_msgs::PoseWithCovariance::ConstPtr& msg_);
			
			bool isReal() {return true;}
		};
		
		typedef std::shared_ptr<KinectLocalizer> KinectLocalizerPtr;
		
		/// Localization sensor for robot using the simulator.
		class SimulatorLocalizer: public iLocalizer
	  	{
		public:
			SimulatorLocalizer(std::string name_);
			
			~SimulatorLocalizer() {};
			
			bool isReal() {return false;}
		};
		
		typedef std::shared_ptr<SimulatorLocalizer> SimulatorLocalizerPtr;
		
	}
}


#endif // iLOCALIZER_H