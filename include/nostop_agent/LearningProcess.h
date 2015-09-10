////////////////////////////////////////////////////
//	LearningProcess.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef LEARNING_PROCESS_H
#define LEARNING_PROCESS_H
#pragma once

#include "ThreadBase.h"

#include "Configuration.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class iGuard;
		class LearningWorld;
		class MonitorReceiver;
		class GuardNeighbours;
	  
		class LearningProcess: public ThreadBase
		{
		protected:
			bool m_update;
			
			ros::NodeHandle m_node;
			ros::Subscriber m_sub;
			
			mutable Mutex1 m_mutex;
			Condition1 m_cond_var;
			bool m_notified;
			
			Configuration m_nextConfiguration;
			
			std::shared_ptr<LearningWorld> m_learning;
			
			std::shared_ptr<MonitorReceiver> m_monitorReceiver;
			std::shared_ptr<GuardNeighbours> m_guardNeighbours;
			
			
			
		protected:
			virtual void run();
		public:
			LearningProcess(std::shared_ptr<LearningWorld> learning_);
			
			~LearningProcess();
			
			void init();
			
			void AgentCall_CallBack(const std_msgs::Bool::ConstPtr & msg_);
			
			double getTimeOFMonitor();
			double getTimeOFNeighbours();
		};

	}
}


#endif // LEARNING_PROCESS_H