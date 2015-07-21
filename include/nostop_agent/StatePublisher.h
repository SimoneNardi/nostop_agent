////////////////////////////////////////////////////
//	StatePublisher.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef STATE_PUBLISHER_H
#define STATE_PUBLISHER_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

// #include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

namespace Robotics 
{
	namespace GameTheory
	{
		class iGuard;
	  
		class StatePublisher: public ThreadBase	  	
		{
		protected:
			std::shared_ptr<iGuard> m_guard;
		  
			ros::NodeHandle m_node;
		  	ros::Publisher m_statePub;
			
// 			tf::TransformBroadcaster m_TFBoradcast;
			ros::Subscriber m_subPose;
					
		protected:
			virtual void run();
			
			void poseCallBack(const geometry_msgs::PoseConstPtr& msg);
			
		public:
			StatePublisher(std::shared_ptr<iGuard> agent_);
			
			~StatePublisher();
		};

	}
}


#endif // GUARD_STATE_PUBLISHER_H