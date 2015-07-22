////////////////////////////////////////////////////
//	iLocalizer.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef iLOCALIZER_H
#define iLOCALIZER_H
#pragma once

#include "ThreadBase.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "ros/ros.h"

#include "Configuration.h"

namespace Robotics
{
	namespace GameTheory
	{
	  	enum ColorName
		{
		    red = 0,
		    blue,
		    green,
		    yellow
		};
		
		/// Localization sensor for robot.
		class iLocalizer
		{
		protected:
		  
		  Configuration m_config;

		  mutable Mutex m_mutex;
		  
		  std::string m_name;
		  
		  ros::NodeHandle m_node;
		  ros::Subscriber m_sub;
		protected:
			virtual void subscribe() = 0;

			virtual void updatePosition() = 0;
			
			virtual void updateOrientation() = 0;
		  		  
		public:
			iLocalizer(std::string name_) : m_name(name_) {};
			
			~iLocalizer() {};
			
			geometry_msgs::Point getPosition() const; 
			
			geometry_msgs::Quaternion getOrientation() const;
			
			geometry_msgs::Twist getTwist() const; 
			
			geometry_msgs::Pose getPose() const;
			
			nav_msgs::Odometry getOdometry() const;
			
			Configuration getConfiguration() const;
		};
		
		/// Localization sensor for robot using the kinect.
		class KinectLocalizer: public iLocalizer
	  	{
		  protected:
		  /// back ball color
		  ColorName m_back;
		  
		  /// front ball color
		  ColorName m_front;
		  	
		protected:
			virtual void subscribe();
			
			virtual void update(geometry_msgs::PoseConstPtr & pose_);
		  
			virtual void updatePosition();
			
			virtual void updateOrientation();
									
		public:
			KinectLocalizer(std::string name_);
			
			~KinectLocalizer() {};
		};
		
		/// Localization sensor for robot using the simulator.
		class SimulatorLocalizer: public iLocalizer
	  	{
		protected:
			virtual void subscribe();
			
			virtual void update(geometry_msgs::PoseConstPtr & pose_);
		  
			virtual void updatePosition();
			
			virtual void updateOrientation();
									
		public:
			SimulatorLocalizer(std::string name_);
			
			~SimulatorLocalizer() {};
		};
		
	}
}


#endif // iLOCALIZER_H