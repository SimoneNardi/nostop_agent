////////////////////////////////////////////////////
//	StateUpdater.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef STATE_UPDATER_H
#define STATE_UPDATER_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class iGuard;
	  
		class StateUpdater: public ThreadBase	  	
		{
		protected:
			std::shared_ptr<iGuard> m_guard;
		  
			ros::NodeHandle m_node;
		  	ros::Publisher m_statePub;
		
		protected:
			virtual void run();
		public:
			StateUpdater(std::shared_ptr<iGuard> agent_);
			
			~StateUpdater();
		};

	}
}


#endif // GUARD_STATE_UPDATER_H