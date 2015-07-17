////////////////////////////////////////////////////
//	iThief.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_THIEF_H
#define I_THIEF_H
#pragma once

#include "guard.h"
#include "iAgent.h"
#include <memory>

#include <geometry_msgs/Quaternion.h>

namespace Robotics
{
	namespace GameTheory
	{
		class iGuard : public iAgent, public Guard
	  	{
		public:
			iGuard() {};
			
			~iGuard() {};
			
		};
	}
}


#endif // I_THIEF_H