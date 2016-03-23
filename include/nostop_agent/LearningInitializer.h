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
	  class Area;
	  
		class LearningInitializer : public ThreadBase
	  	{
		  int m_id;
		  std::string m_name;
		  
		  std::shared_ptr<Area> m_area;
		  
		  mutable Mutex m_mutex;
		  
		  enum RobotType
		  {
		    REAL = 0,
		    REAL_MAG = 1,
		    SIMULATED = 2,
		    IROBOT_REAL = 3,
		    IROBOT_REAL_MAG = 4,
		    IROBOT_SIMULATED = 5
		  } m_type;
		  
		protected:
			virtual void run();
		public:
			LearningInitializer(std::string const& name_ = "");
			
			~LearningInitializer();
			
			bool isInitialized() const;
			
			int getID() const;
			
			void setType(int const& type_);
			
			void setName(std::string const& name_);
			
			void setSpace(std::shared_ptr<Area>);
			
			std::shared_ptr<Area> getSpace() const;
		};
	}
}


#endif // LEARNING_INITIALIZER_H