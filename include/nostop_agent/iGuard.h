////////////////////////////////////////////////////
//	iGuard.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_GUARD_H
#define I_GUARD_H
#pragma once

#include "iAgent.h"
#include <memory>

#include <geometry_msgs/Quaternion.h>

#include "nostop_agent/GuardSensorCtrl.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;
		class Guard;
		class AgentPosition;
		class GuardNeighbours;
		class MonitorReceiver;
		
		class iGuard : public iAgent
	  	{
		protected:
			///
			std::shared_ptr<LearningWorld> m_learninigWorld;
			///
			std::shared_ptr<Guard> m_LGuard;
			
			  
			std::shared_ptr<MonitorReceiver> m_monitorReceiver;
			std::shared_ptr<GuardNeighbours> m_guardNeighbours;
		  
		public:
			//AgentPosition getCurrentAgentPosition();
			
			nostop_agent::GuardSensorCtrl getCameraControl();

		public:
			iGuard();
			
			~iGuard() {};
			
		};
	}
}


#endif // I_GUARD_H