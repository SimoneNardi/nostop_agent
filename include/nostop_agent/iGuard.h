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

#include "LearningInitializer.h"
#include <LearningWorld.h>

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;
		class Guard;
		class AgentPosition;
		class GuardNeighbours;
		class MonitorReceiver;
		class Area;

		class iGuard : public iAgent
	  	{
		protected:
			///
			std::shared_ptr<LearningWorld> m_learninigWorld;
			///
			std::shared_ptr<Guard> m_LGuard;
			
			  
			std::shared_ptr<MonitorReceiver> m_monitorReceiver;
			std::shared_ptr<GuardNeighbours> m_guardNeighbours;
			std::shared_ptr<LearningInitializer> m_learningInit;
			
			LEARNING m_algorithmFLAG;
			
			nostop_agent::GuardSensorCtrl m_currentControl;
			
		public:
			//AgentPosition getCurrentAgentPosition();
			
			nostop_agent::GuardSensorCtrl getCameraControl();
			
			std::shared_ptr<LearningInitializer> getLearningInitializer() {return m_learningInit;}

		public:
		  
		  void setCameraCtrl(nostop_agent::GuardSensorCtrl l_ctrl);
		  
		  void createLearningAlgorithm( std::shared_ptr<Area>  l_area );
		  
		  void setTargetConfigurationToCenterOfSquare(geometry_msgs::Point const& target_);
		  
		  void setGuardPtr(std::shared_ptr<Guard> lGuard_);
			
		public:
			iGuard();
			
			~iGuard() {};
			
		};
	}
}


#endif // I_GUARD_H