////////////////////////////////////////////////////
//	GuardStateUpdater.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef GUARD_STATE_UPDATER_H
#define GUARD_STATE_UPDATER_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class Guard;
	  
		class GuardStateUpdater: public ThreadBase	  	
		{
		protected:
			std::shared_ptr<Guard> m_guard;
		  
			ros::NodeHandle m_node;
		  	ros::Publisher m_statePub;
		
		protected:
			virtual void run();
		public:
			GuardStateUpdater(std::shared_ptr<Guard> agent_);
			
			~GuardStateUpdater();
		};

	}
}


#endif // GUARD_STATE_UPDATER_H