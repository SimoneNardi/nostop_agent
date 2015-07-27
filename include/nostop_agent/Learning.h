////////////////////////////////////////////////////
//	Learning.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef LEARNING_H
#define LEARNING_H
#pragma once

#include <memory>
#include "ThreadBase.h"

namespace Robotics 
{
	namespace GameTheory
	{	 
	  class iAgent;
	  
		class Learning : public ThreadBase
	  	{
		  AgentPosition m_nextPosition;
		  
		  
		  
		protected:
			virtual void run();
		public:
			Learning();
			
			~Learning() {};
		};
	}
}


#endif // LEARNING_H