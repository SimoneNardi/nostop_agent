////////////////////////////////////////////////////
//	GuardProcess.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef GUARD_PROCESS_H
#define GUARD_PROCESS_H
#pragma once

#include "AgentProcess.h"
#include "nostop_agent/GuardSensorCtrl.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class Area;
		
		class GuardProcess : public AgentProcess
	  	{
		public:
			void setCamera(nostop_agent::GuardSensorCtrl & camera_);
		  
			GuardProcess(std::string name_);
			
			~GuardProcess();
			
			virtual void start();
			
			virtual bool isReady();
			
			void createLearningAlgorithm( );
			
			void setCamera(double max_dist);

			void setRobotAlgorithm(std::string alg_);
			
			void spin();
			
			void setAreaForInitialization( std::shared_ptr<Area> area_ );
		};

	}
}


#endif // COLLECT_PLAYERS_H