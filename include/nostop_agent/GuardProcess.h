////////////////////////////////////////////////////
//	GuardProcess.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef GUARD_PROCESS_H
#define GUARD_PROCESS_H
#pragma once

#include "agent.h"

#include "ros/ros.h"

#include "nostop_agent/GuardStateData.h"

#include "ThreadBase.h"
#include "AgentProcess.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class StateUpdater;
		class StatePublisher;
		class MotorControl;
	  
		class GuardProcess : public AgentProcess
	  	{
			std::shared_ptr<StatePublisher> m_statePublisher;
			std::shared_ptr<StateUpdater> m_stateUpdater;
		  
		public:
			GuardProcess();
			
			~GuardProcess();
		};

	}
}


#endif // COLLECT_PLAYERS_H