////////////////////////////////////////////////////
//	iAgent.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_AGENT_H
#define I_AGENT_H
#pragma once

#include "agent.h"
#include "Configuration.h"
#include <memory>
#include "iLocalizer.h"

#include <nav_msgs/Odometry.h>

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
			
			Mutex m_mutex;
			
			std::shared_ptr<Agent> m_LAgent;
			
			std::string m_name;
					
		public:
		  
			void setCurrentConfiguration( nav_msgs::Odometry & odometry_ );
		  
			void setCurrentConfiguration( geometry_msgs::Pose & pose_ );
			
			void setCurrentConfiguration( Configuration & config_ );
			
			void setCurrentOrientation(geometry_msgs::Quaternion & orientation_);
			
			void setCurrentPosition(geometry_msgs::Point & currentPosition_);
			
		public:
			iAgent() : m_localizer(nullptr) {};
			
			~iAgent() {};
		
			/// Create the link between this agent and the agent of the learning
			void setAgentPtr(std::shared_ptr<Agent> agent_);						
			/// Create a kinect localizer
			void setKinectLocalizer(std::string name_);
			
			/// Create a simulator localizer
			void setSimulatorLocalizer(std::string name_);
			
			/// get current angular speed in world TF
			double getCurrentAngularSpeed();
	
			/// get current linear speed in world TF
			double getCurrentLinearSpeed();
			
			Configuration getCurrentConfiguration() {return m_currentConfiguration;}
			Configuration getTargetConfiguration() {return m_targetConfiguration;}
			
			/// Get the robot name
			std::string getName() {return m_name;}
			
			bool isArrived();
			
			void setStandByStatus();
			
			void setActiveStatus();
			
			void setName(std::string name_);
		};
	}
}


#endif // I_AGENT_H