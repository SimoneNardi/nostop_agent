////////////////////////////////////////////////////
//	iGuard.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_GUARD_H
#define I_GUARD_H
#pragma once

#include "guard.h"
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


#endif // I_GUARD_H