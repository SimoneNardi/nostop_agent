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
		  Configuration (geometry_msgs::Pose & pose_);
		  Configuration (nav_msgs::Odometry& odom_);
		  
		  void setPosition(geometry_msgs::Point & position_);
		  void setOrientation(geometry_msgs::Quaternion & orientation_);
		  void setPose(geometry_msgs::Pose & pose_);
		  
		  geometry_msgs::Point getPosition() const;
		  geometry_msgs::Quaternion getOrientation() const;
		  
		  geometry_msgs::Pose getPose() const;
		  geometry_msgs::Twist getTwist() const;
		  
		  nav_msgs::Odometry getOdometry() const;
		  
		  bool equals(const Configuration& other_) const;
		};
	}
}


#endif // I_AGENT_H