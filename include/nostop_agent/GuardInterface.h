////////////////////////////////////////////////////
//	GuardInterface.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef GUARD_INTERFACE_H
#define GUARD_INTERFACE_H
#pragma once

#include "agent.h"

#include "ros/ros.h"

#include "nostop/GuardStateData.h"

#include "ThreadBase.h"
#include "AgentInterface.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class GuardStateUpdater;
		class GuardStatePublisher;
		class AgentMotorControl;
	  
		class GuardInterface : public AgentInterface
	  	{
			std::shared_ptr<GuardStatePublisher> m_statePublisher;
			std::shared_ptr<GuardStateUpdater> m_stateUpdater;
			std::shared_ptr<AgentMotorControl> m_motorControl;
		  
		public:
			GuardInterface(int id_, AgentPosition initial_position_);
			
			~GuardInterface();
		};

	}
}


#endif // COLLECT_PLAYERS_H