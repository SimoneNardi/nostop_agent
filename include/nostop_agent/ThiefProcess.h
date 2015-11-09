////////////////////////////////////////////////////
//	ThiefProcess.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef THIEF_PROCESS_H
#define THIEF_PROCESS_H
#pragma once

#include "AgentProcess.h"
#include "nostop_agent/GuardSensorCtrl.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class ThiefProcess : public AgentProcess
	  	{
		  int m_id;
		  
		public:
			ThiefProcess(std::string const& name_, int id_);
			
			~ThiefProcess();
			
			virtual void start();
			
			virtual bool isReady();
		};

	}
}


#endif // COLLECT_PLAYERS_H