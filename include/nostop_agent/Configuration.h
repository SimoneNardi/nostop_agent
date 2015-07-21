////////////////////////////////////////////////////
//	Configuration.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#pragma once

#include <nav_msgs/Odometry.h>

namespace Robotics
{
	namespace GameTheory
	{
		class Configuration
		{
		  protected:
			nav_msgs::Odometry m_odom;
						
		public:
		  Configuration();
		  Configuration (geometry_msgs::PoseConstPtr & pose_);
		  Configuration (nav_msgs::OdometryConstPtr& odom_);
		  
		  void setPosition(geometry_msgs::Point & position_);
		  void setOrientation(geometry_msgs::Quaternion & orientation_);
		  void setPose(geometry_msgs::Pose & pose_);
		  
		  geometry_msgs::Point getPosition();
		  geometry_msgs::Quaternion getOrientation();
		  
		  geometry_msgs::Pose getPose();
		  geometry_msgs::Twist getTwist();
		  
		  nav_msgs::Odometry getOdometry();
		};
		
		bool operator==(const Configuration& lhs, const Configuration& rhs);
		bool operator!=(const Configuration& lhs, const Configuration& rhs);
	}
}


#endif // I_AGENT_H