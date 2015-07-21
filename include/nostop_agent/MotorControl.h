////////////////////////////////////////////////////
//	MotorControl.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class iAgent;
	  
		class MotorControl: public ThreadBase	  	
		{
		protected:
			std::shared_ptr<iAgent> m_agent;
		  
			ros::NodeHandle m_node;
		  	ros::Publisher m_controlPub;
		
		protected:
			virtual void run();
		public:
			MotorControl(std::shared_ptr<iAgent> agent_);
			
			~MotorControl();
		};

	}
}


#endif // AGENT_MOTOR_CONTROL_H