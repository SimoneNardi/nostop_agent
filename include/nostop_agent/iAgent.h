////////////////////////////////////////////////////
//	iAgent.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_AGENT_H
#define I_AGENT_H
#pragma once

#include "agent.h"

#include "Threads.h"

#include "Configuration.h"
#include "iLocalizer.h"

#include <nav_msgs/Odometry.h>

#include <memory>
#include <string>

namespace Robotics
{
	namespace GameTheory
	{	  
		/// Tutto ciò che serve per la guida del robot!
		class iAgent
	  	{
		protected:
			Configuration m_currentConfiguration;
			Configuration m_targetConfiguration;
			
			std::shared_ptr<iLocalizer> m_localizer;
			
			std::shared_ptr<Agent> m_LAgent;
			
			std::string m_name;
			
			Mutex m_mutex;
		public:
		  		
			void computeConfigurationToTarget();
		  
		  	void updateTargetConfiguration(geometry_msgs::Point & newTarget_);
			
			void setCurrentConfiguration( Configuration & config_ );
			
			geometry_msgs::Twist getCurrentConfigurationTwist();
			
			geometry_msgs::Point getCurrentConfigurationPosition();
			//////////////////////////////
		  
			void updateCurrentOdometry( nav_msgs::Odometry & odometry_ );
		  
			void updateCurrentPose( geometry_msgs::Pose & pose_ );
			
			void updateCurrentTwist( geometry_msgs::Twist & twist_ );
			
			void updateCurrentOrientation(geometry_msgs::Quaternion & orientation_);
			
			void updateCurrentPosition(geometry_msgs::Point & currentPosition_);
			
		public:
			iAgent() : m_localizer(nullptr), m_LAgent(nullptr) {};
			
			~iAgent() {};
		
			/// Create the link between this agent and the agent of the learning
			void setAgentPtr(std::shared_ptr<Agent> agent_);						
			/// Create a kinect localizer
			void setKinectLocalizer(std::string name_);
			
			/// Create a simulator localizer
			void setSimulatorLocalizer(std::string name_);
			
			Configuration getCurrentConfiguration() {return m_currentConfiguration;}
			Configuration getTargetConfiguration() {return m_targetConfiguration;}
			
			/// Get the robot name
			std::string getName() {return m_name;}
			
			bool isArrived();
			
			void setStandByStatus();
			
			void setActiveStatus();
			
			void setName(std::string name_);
			
			std::shared_ptr<iLocalizer> getLocalizer();
			
			/// Wrapper Agent:
			
			void setLearnAgent(std::shared_ptr<Agent> lAgent_);
			
			int getID();
			
			Agent::Status getStatus();
			
		protected:
			  void goForward();
			  void goBackward();
			  void rotateLeft();
			  void rotateRight();
		};
	}
}


#endif // I_AGENT_H