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

#include <geometry_msgs/Pose.h>

namespace Robotics 
{
	namespace GameTheory
	{
		class iAgent;
	  
		class StatePublisher: public ThreadBase	  	
		{
		protected:
			std::shared_ptr<iAgent> m_agent;

			ros::NodeHandle m_node;
		  	ros::Publisher m_statePub;
			ros::Publisher m_posePub;
			
			ros::Subscriber m_subPose;
		protected:
			virtual void run() = 0;
			
		public:
			StatePublisher(std::shared_ptr<iAgent> agent_);
			
			~StatePublisher();
		};

	}
}


#endif // GUARD_STATE_PUBLISHER_H