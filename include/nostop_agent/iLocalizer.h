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
		  
		  std::string m_name;
		  
		  ros::NodeHandle m_node;
		  ros::Subscriber m_sub;

		public:
			iLocalizer(std::string name_) : m_name(name_) {};
			
			~iLocalizer() {};
			
			geometry_msgs::Point getPosition() const; 
			
			geometry_msgs::Quaternion getOrientation() const;
			
			geometry_msgs::Twist getTwist() const; 
			
			geometry_msgs::Pose getPose() const;
			
			nav_msgs::Odometry getOdometry() const;
			
			Configuration getConfiguration() const;
			
			void updatePose(const geometry_msgs::Pose::ConstPtr  & pose_);
			
			void subscribeTopic();
		};
		
		/// Localization sensor for robot using the kinect.
		class KinectLocalizer: public iLocalizer
	  	{
		public:
			KinectLocalizer(std::string name_);
			
			~KinectLocalizer() {};
		};
		
		/// Localization sensor for robot using the simulator.
		class SimulatorLocalizer: public iLocalizer
	  	{
		public:
			SimulatorLocalizer(std::string name_);
			
			~SimulatorLocalizer() {};
		};
	}
}


#endif // iLOCALIZER_H