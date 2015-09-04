////////////////////////////////////////////////////
//	LearningInitializer.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef LEARNING_INITIALIZER_H
#define LEARNING_INITIALIZER_H
#pragma once

#include "ThreadBase.h"

namespace Robotics 
{
	namespace GameTheory
	{	 
	  class iAgent;
	  
		class LearningInitializer : public ThreadBase
	  	{
		  int m_id;
		  std::string m_name;
		  
		  mutable Mutex m_mutex;
		  
		protected:
			virtual void run();
		public:
			LearningInitializer(std::string name_);
			
			~LearningInitializer();
			
			bool isInitialized() const;
			
			int getID() const;
		};
	}
}


#endif // LEARNING_INITIALIZER_H