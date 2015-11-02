////////////////////////////////////////////////////
//	Configuration.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#pragma once

#include <nav_msgs/Odometry.h>

#include "Math.h"

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
		  Configuration (geometry_msgs::Pose const& pose_);
		  Configuration (nav_msgs::Odometry const& odom_);
		  Configuration (geometry_msgs::Point const& point_);
		  
		  void setPosition(const geometry_msgs::Point & position_);
		  void setOrientation(const geometry_msgs::Quaternion & orientation_);
		  void setPose(const geometry_msgs::Pose & pose_);
		  void setTwist(const geometry_msgs::Twist & twist_);
		  void setOdometry(const nav_msgs::Odometry & odometry_);
		  void setConfiguration(const Configuration & config_);
		  
		  geometry_msgs::Point getPosition() const;
		  geometry_msgs::Quaternion getOrientation() const;
		  
		  geometry_msgs::Pose getPose() const;
		  geometry_msgs::Twist getTwist() const;
		  
		  nav_msgs::Odometry getOdometry() const;
		  
		  bool equals(const Configuration& other_, double tolerance = Math::TOLERANCE) const;
		};
	}
}


#endif // CONFIGURATION_H