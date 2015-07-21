////////////////////////////////////////////////////
//	iLocalizer.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef iLOCALIZER_H
#define iLOCALIZER_H
#pragma once

#include "Threads.h"

namespace Robotics
{
	namespace GameTheory
	{
	  	enum ColorName
		{
		    red = 0,
		    blue,
		    green,
		    yellow
		};
		
		enum LocalizerType
		{
		    kinect = 0, 
		    simulator
		};
		
		/// Localization sensor for robot.
		class iLocalizer
		{
		protected:
		  
		  IDSReal2D m_position;
		  
		  double m_orientation;

		  Mutex m_mutex;
		  		  
		public:
			iLocalizer() {};
			
			~iLocalizer() {};
			
			IDSReal2D getPosition(); 
			
			double getOrientation();
		};
		
		/// Localization sensor for robot using the kinect.
		class KinectLocalizer: public iLocalizer
	  	{
		  /// back ball color
		  ColorName m_back;
		  
		  /// front ball color
		  ColorName m_front;
		  	
		protected:
		  
			void updatePosition();
			
			void updateOrientation();
									
		public:
			KinectLocalizer(ColorName back_, ColorName front_) : m_back(back_), m_front(front_) {};
			
			~KinectLocalizer() {};
		};
		
		/// Localization sensor for robot using the simulator.
		class SimulatorLocalizer: public iLocalizer
	  	{
		    std::string m_name;
		  
		protected:
		  
			void updatePosition();
			
			void updateOrientation();
									
		public:
			SimulatorLocalizer(std::string name_) : m_name(name_) {};
			
			~SimulatorLocalizer() {};
		};
		
	}
}


#endif // iLOCALIZATION_H