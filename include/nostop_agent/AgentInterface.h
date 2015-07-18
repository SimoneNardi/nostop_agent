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

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace Robotics 
{
	namespace GameTheory
	{
		class AgentInterface
	  	{
		protected:
			std::shared_ptr<Agent> m_agent;
			
			geometry_msgs::Quaternion m_currentOrientation;
		public:
			AgentInterface() {};
			
			~AgentInterface() {};
			
			std::shared_ptr<Agent> getAgent() {return m_agent;}
			
			void setCurrentOrientation(geometry_msgs::Quaternion & orientation_) {m_currentOrientation = orientation_;}
		};
		
		AgentPosition computeAgentPosition(geometry_msgs::Point & point_);
	}
}


#endif // COLLECT_PLAYERS_H