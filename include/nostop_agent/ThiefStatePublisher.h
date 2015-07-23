////////////////////////////////////////////////////
//	ThiefStatePublisher.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef THIEF_STATE_PUBLISHER_H
#define THIEF_STATE_PUBLISHER_H
#pragma once

#include "StatePublisher.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class iThief;
	  
		class ThiefStatePublisher: public StatePublisher
		{
		protected:
			std::shared_ptr<iThief> m_thief;
		  			
		protected:
			virtual void run();
		public:
			ThiefStatePublisher(std::shared_ptr<iAgent> agent_);
			
			~ThiefStatePublisher();
		};

	}
}


#endif // THIEF_STATE_PUBLISHER_H