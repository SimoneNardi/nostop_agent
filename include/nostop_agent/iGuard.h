////////////////////////////////////////////////////
//	iGuard.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_GUARD_H
#define I_GUARD_H
#pragma once

#include "iAgent.h"
#include "guard.h"
#include <memory>

#include <geometry_msgs/Quaternion.h>

namespace Robotics 
{
	namespace GameTheory
	{
		class iGuard : public iAgent
	  	{
		  protected:
		  std::shared_ptr<Guard> m_LGuard;
		  
		public:
			AgentPosition getCurrentAgentPosition();

		public:
			iGuard() {};
			
			~iGuard() {};
			
		};
	}
}


#endif // I_GUARD_H