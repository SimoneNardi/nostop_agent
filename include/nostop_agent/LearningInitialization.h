////////////////////////////////////////////////////
//	LearningInitialization.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef LEARNING_INITIALIZATION_H
#define LEARNING_INITIALIZATION_H
#pragma once

#include <memory>
#include "ThreadBase.h"

namespace Robotics 
{
	namespace GameTheory
	{	 
	  class iAgent;
	  
		class LearningInitialization : public ThreadBase
	  	{
		protected:
		  std::shared_ptr<iAgent> m_agent;
		protected:
		  virtual void run();
		public:
			LearningInitialization() {};
			
			~LearningInitialization() {};
		};
	}
}


#endif // LEARNING_INITIALIZATION_H