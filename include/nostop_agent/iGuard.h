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

#include "learningWorld.h"

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
		class LearningProcess;
		class LearningInitializer;

		class iGuard : public iAgent
	  	{
		protected:
			/// Learning Initializer
			std::shared_ptr<LearningInitializer> m_learningInit;
			/// Learning
			std::shared_ptr<LearningProcess> m_learning;
			///
			std::shared_ptr<Guard> m_LGuard;
			
			LEARNING m_algorithmFLAG;
			
			nostop_agent::GuardSensorCtrl m_currentControl;
			
		public:
			nostop_agent::GuardSensorCtrl getCameraControl();
			
			std::shared_ptr<LearningInitializer> getLearningInitializer() {return m_learningInit;}

		public:
		  
		  void setCameraCtrl(nostop_agent::GuardSensorCtrl ctrl_);
		  
		  void createLearningAlgorithm( std::shared_ptr<Area>  area_ );
		  
		  void setTargetConfigurationToCenterOfSquare(geometry_msgs::Point const& target_);
		  
		  void setGuardPtr(std::shared_ptr<Guard> lGuard_);
		  
		  void setRobotAlgorithm(std::string alg_);
			
		public:
			iGuard();
			
			~iGuard() {};
			
		};
		
		typedef std::shared_ptr<iGuard> iGuardPtr;
	}
}


#endif // I_GUARD_H