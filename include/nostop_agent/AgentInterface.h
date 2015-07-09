////////////////////////////////////////////////////
//	AgentInterface.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_INTERFACE_H
#define AGENT_INTERFACE_H
#pragma once

#include "agent.h"
#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class AgentInterface
	  	{
		protected:
			std::shared_ptr<Agent> m_agent;
		public:
			AgentInterface() {};
			
			~AgentInterface() {};
			
			std::shared_ptr<Agent> getAgent() {return m_agent;}
		};

	}
}


#endif // COLLECT_PLAYERS_H