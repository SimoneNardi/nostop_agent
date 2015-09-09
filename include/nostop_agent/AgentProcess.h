////////////////////////////////////////////////////
//	AgentProcess.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_PROCESS_H
#define AGENT_PROCESS_H
#pragma once

#include "ros/ros.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
	  class MotorControl;
	  class iAgent;
	  class StatePublisher;
	  class StateUpdater;
	  
		class AgentProcess
	  	{
		protected:
		  std::shared_ptr<iAgent> m_agent;
		  
		  std::shared_ptr<MotorControl> m_motorControl;
		  std::shared_ptr<StatePublisher> m_statePublisher;
		  std::shared_ptr<StateUpdater> m_stateUpdater;
		  
		  ros::NodeHandle m_node;
		  ros::ServiceClient m_notifyStatus;
		  
		public:
			AgentProcess() {};
			
			~AgentProcess() {};
			
			void setRobotName(std::string name_);
			void setRobotAlgorithm(std::string alg_);
			
			virtual void init();
			
			virtual bool isReady() = 0;
			
			void setKinectLocalizer();
			void setSimulatorLocalizer();
			
		protected:
			void reachTargetConfiguration();
			
			// Fail if simulator is not responding
			bool notifyStatus();
		};
	}
}


#endif // AGENT_PROCESS_H