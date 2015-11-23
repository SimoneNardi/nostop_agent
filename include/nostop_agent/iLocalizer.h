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
		};
		
		typedef std::shared_ptr<iLocalizer> iLocalizerPtr;
		
		/// Localization sensor for robot using the kinect.
		class KinectLocalizer: public iLocalizer
	  	{
		public:
			KinectLocalizer(std::string name_);
			
			~KinectLocalizer() {};
		};
		
		typedef std::shared_ptr<KinectLocalizer> KinectLocalizerPtr;
		
		/// Localization sensor for robot using the simulator.
		class SimulatorLocalizer: public iLocalizer
	  	{
		public:
			SimulatorLocalizer(std::string name_);
			
			~SimulatorLocalizer() {};
		};
		
		typedef std::shared_ptr<SimulatorLocalizer> SimulatorLocalizerPtr;
		
	}
}


#endif // iLOCALIZER_H