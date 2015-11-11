////////////////////////////////////////////////////
//	iGuard.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_GUARD_H
#define I_GUARD_H
#pragma once

#include "iAgent.h"
#include <memory>

#include <geometry_msgs/Quaternion.h>

#include "nostop_agent/GuardSensorCtrl.h"

#include "learningWorld.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;
		class Guard;
		class AgentPosition;
		class GuardNeighbours;
		class MonitorReceiver;
		class Area;
		class LearningProcess;
		class LearningInitializer;

		class iGuard : public iAgent
	  	{
		protected:
			/// Learning Initializer
			std::shared_ptr<LearningInitializer> m_learningInit;
			/// Learning
			std::shared_ptr<LearningProcess> m_learning;
			///
			std::shared_ptr<Guard> m_LGuard;
			
			LEARNING m_algorithmFLAG;
			
			ros::ServiceClient m_notifyStatus;
			
			nostop_agent::GuardSensorCtrl m_currentControl;
		public:
			nostop_agent::GuardSensorCtrl getCameraControl();
			
			std::shared_ptr<LearningInitializer> getLearningInitializer() {return m_learningInit;}

		public:
		  
		  virtual void setName(std::string const& name_);
		  
		  void setCameraCtrl(nostop_agent::GuardSensorCtrl ctrl_);
		  
		  void setCameraCtrl(CameraPosition const& l_ctrl);
		  
		  void createLearningAlgorithm( );
		  
		  void setTargetConfigurationToCenterOfSquare(geometry_msgs::Point const& target_);
		  
		  void setGuardPtr(std::shared_ptr<Guard> lGuard_);
		  
		  void setRobotAlgorithm(std::string alg_);
		  
		  /// Info From Monitor Sensor (Learning Benefit and Neighbours) and update learning
		  void waitForNewsFromMonitor(ros::Time const& time_);
		  		  
		  /// Start Learning Algorithm
		  void startLearning();
			
		public:
			iGuard();
			
			~iGuard() {};
			
		protected: 
			virtual bool notifyStatus();
		};
		
		typedef std::shared_ptr<iGuard> iGuardPtr;
	}
}


#endif // I_GUARD_H