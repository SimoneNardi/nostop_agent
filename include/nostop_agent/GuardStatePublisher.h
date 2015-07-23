////////////////////////////////////////////////////
//	GuardStatePublisher.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef GUARD_STATE_PUBLISHER_H
#define GUARD_STATE_PUBLISHER_H
#pragma once

#include "StatePublisher.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class iGuard;
	  
		class GuardStatePublisher: public StatePublisher
		{
		protected:
			std::shared_ptr<iGuard> m_guard;
		  			
		protected:
			virtual void run();
		public:
			GuardStatePublisher(std::shared_ptr<iAgent> agent_);
			
			~GuardStatePublisher();
		};

	}
}


#endif // GUARD_STATE_PUBLISHER_H