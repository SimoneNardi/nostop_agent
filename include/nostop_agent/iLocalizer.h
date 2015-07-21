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
		
		enum LocalizerType
		{
		    kinect = 0, 
		    simulator
		};
		
		/// Localization sensor for robot.
		class iLocalizer
		{
		protected:
		  
		  Configuration m_config;

		  Mutex m_mutex;
		  
		  std::string m_name;
		  
		  ros::NodeHandle m_node;
		  ros::Publisher m_sub;
		protected:

			virtual void updatePosition() = 0;
			
			virtual void updateOrientation() = 0;
		  		  
		public:
			iLocalizer(std::string name_) : m_name(name_) {};
			
			~iLocalizer() {};
			
			geometry_msgs::Point getPosition(); 
			
			geometry_msgs::Quaternion getOrientation();
			
			Configuration getConfiguration();
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
			virtual void update(geometry_msgs::PoseConstPtr & pose_);
		  
			virtual void updatePosition();
			
			virtual void updateOrientation();
									
		public:
			SimulatorLocalizer(std::string name_);
			
			~SimulatorLocalizer() {};
		};
		
	}
}


#endif // iLOCALIZATION_H