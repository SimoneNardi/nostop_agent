////////////////////////////////////////////////////
//	GuardStatePublisher.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef GUARD_STATE_PUBLISHER_H
#define GUARD_STATE_PUBLISHER_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

namespace Robotics 
{
	namespace GameTheory
	{
		class Guard;
	  
		class GuardStatePublisher: public ThreadBase	  	
		{
		protected:
			std::shared_ptr<Guard> m_guard;
		  
			ros::NodeHandle m_node;
		  	ros::Publisher m_statePub;
			
			tf::TransformBroadcaster m_TFBoradcast;
			ros::Subscriber m_subPose;
					
		protected:
			virtual void run();
			
			void poseCallBack(const geometry_msgs::PoseConstPtr& msg);
			
		public:
			GuardStatePublisher(std::shared_ptr<Guard> agent_);
			
			~GuardStatePublisher();
		};

	}
}


#endif // GUARD_STATE_PUBLISHER_H