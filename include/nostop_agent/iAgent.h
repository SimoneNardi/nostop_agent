////////////////////////////////////////////////////
//	iAgent.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_AGENT_H
#define I_AGENT_H
#pragma once

#include "agent.h"
#include <memory>

#include <geometry_msgs/Quaternion.h>

namespace Robotics
{
	namespace GameTheory
	{
		/// Tutto ciò che serve per la guida del robot!
		class iAgent
	  	{
		protected:
			geometry_msgs::Quaternion m_currentOrientation;
			
		public:
			iAgent() {};
			
			~iAgent() {};
			
			void setCurrentOrientation(geometry_msgs::Quaternion & orientation_) {m_currentOrientation = orientation_;}
		};
		
		AgentPosition computeAgentPosition(geometry_msgs::Point & point_);
	}
}


#endif // I_AGENT_H