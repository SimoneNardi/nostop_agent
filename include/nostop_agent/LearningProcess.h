////////////////////////////////////////////////////
//	LearningProcess.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef LEARNING_PROCESS_H
#define LEARNING_PROCESS_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class iGuard;
	  
		class LearningProcess: public ThreadBase
		{
		protected:
			int m_id;
			bool m_update;
			
			ros::NodeHandle m_node;
			ros::Subscriber m_sub;
			
			Mutex m_mutex;
		
		protected:
			virtual void run();
		public:
			LearningProcess(int id_);
			
			~LearningProcess();
			
			void init();
			
			void AgentCall_CallBack(const std_msgs::Bool::ConstPtr & msg_);
		};

	}
}


#endif // LEARNING_PROCESS_H