////////////////////////////////////////////////////
//	StateUpdater.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef STATE_UPDATER_H
#define STATE_UPDATER_H
#pragma once

#include "ThreadBase.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class iAgent;
	  
		class StateUpdater: public ThreadBase
		{
		protected:
			std::shared_ptr<iAgent> m_agent;
		
		protected:
			virtual void run();
		public:
			StateUpdater(std::shared_ptr<iAgent> agent_);
			
			~StateUpdater();
		};

	}
}


#endif // GUARD_STATE_UPDATER_H