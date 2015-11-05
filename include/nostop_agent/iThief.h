////////////////////////////////////////////////////
//	iThief.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_THIEF_H
#define I_THIEF_H
#pragma once

#include "iAgent.h"
#include <memory>

#include <geometry_msgs/Quaternion.h>

#include "nostop_agent/GuardSensorCtrl.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;
		class Thief;
		
		class iThief : public iAgent
	  	{
		protected:
			///
			std::shared_ptr<LearningWorld> m_learninigWorld;
			///
			std::shared_ptr<Thief> m_LThief;
			
		public:
			iThief();
			
			~iThief() {};
			
			virtual void setName(std::string const& name_);
		};
		
		typedef std::shared_ptr<iThief> iThiefPtr;
	}
}


#endif // I_THIEF_H