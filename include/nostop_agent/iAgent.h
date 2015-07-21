////////////////////////////////////////////////////
//	iAgent.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef I_AGENT_H
#define I_AGENT_H
#pragma once

#include "agent.h"
#include <memory>
#include "iLocalizer.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace Robotics
{
	namespace GameTheory
	{
		class Configuration
		{
			geometry_msgs::Quaternion m_orientation;
			geometry_msgs::Point m_position;
			
		public:
		  Configuration();
		  
		  void setPosition(geometry_msgs::Point & position_);
		  void setOrientation(geometry_msgs::Quaternion & orientation_);
		  
		  geometry_msgs::Point getPosition();
		  geometry_msgs::Quaternion getOrientation();
		};
		
		bool operator==(const Configuration& lhs, const Configuration& rhs);
		bool operator!=(const Configuration& lhs, const Configuration& rhs);
	  
		/// Tutto ciò che serve per la guida del robot!
		class iAgent
	  	{
		protected:
			Configuration m_currentConfiguration;
			Configuration m_targetConfiguration;
			
			std::shared_ptr<iLocalizer> m_localizer;
			
			Mutex m_mutex;
			
			std::shared_ptr<Agent> m_agent;
		protected:
			AgentPosition getCurrentPosition();
			
		public:
			
			void setCurrentOrientation(geometry_msgs::Quaternion & orientation_);
			
			void setCurrentPosition(geometry_msgs::Point & currentPosition_);
			
		public:
			iAgent() : m_localizer(nullptr) {};
			
			~iAgent() {};
		
			/// Create the link between this agent and the agent of the learning
			void setAgentPtr(std::shared_ptr<Agent> agent_);						
			/// Create a kinect localizer
			void setLocalizer(ColorName back_, ColorName front_);
			
			/// Create a simulator localizer
			void setLocalizer(std::string name_);
			
			/// get current angular speed in world TF
			double getCurrentAngularSpeed();
	
			/// get current linear speed in world TF
			double getCurrentLinearSpeed();
	
		};
	}
}


#endif // I_AGENT_H