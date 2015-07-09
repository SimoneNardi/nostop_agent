////////////////////////////////////////////////////
//	AgentMotorControl.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_MOTOR_CONTROL_H
#define AGENT_MOTOR_CONTROL_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class Agent;
	  
		class AgentMotorControl: public ThreadBase	  	
		{
		protected:
			std::shared_ptr<Agent> m_agent;
		  
			ros::NodeHandle m_node;
		  	ros::Publisher m_controlPub;
		
		protected:
			virtual void run();
		public:
			AgentMotorControl(std::shared_ptr<Agent> agent_);
			
			~AgentMotorControl();
		};

	}
}


#endif // AGENT_MOTOR_CONTROL_H