////////////////////////////////////////////////////
//	AgentProcess.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_PROCESS_H
#define AGENT_PROCESS_H
#pragma once

#include "MotorControl.h"
#include "iAgent.h"
#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class AgentProcess
	  	{
		protected:
		  std::shared_ptr<iAgent> m_agent;
		  
		  std::shared_ptr<MotorControl> m_motorControl;
		  
		public:
			AgentProcess() {};
			
			~AgentProcess() {};
			
			void setRobotName(std::string name_);
			void setRobotAlgorithm(std::string alg_);
			
			void setCamera(Robotics::GameTheory::CameraPosition & camera_);
			void setID(int id_);
		};
	}
}


#endif // AGENT_PROCESS_H