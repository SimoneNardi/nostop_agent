////////////////////////////////////////////////////
//	GuardNeighbours.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef GUARD_NEIGHBOURS_H
#define GUARD_NEIGHBOURS_H
#pragma once

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>

#include "Threads.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class WorldMap;
	  
		class GuardNeighbours
		{
		protected:
			ros::NodeHandle m_node;
		  	ros::Subscriber m_sub;
			
			std::shared_ptr<WorldMap> m_data;
			
			ros::Time m_time;
			
			bool m_isUpdated;
			
			mutable Mutex1 m_mutex;
		protected:
		
			void UpdateNeighboursCallBack(const nav_msgs::OccupancyGrid::ConstPtr & msg);
			
		public:
			GuardNeighbours();
			
			~GuardNeighbours();
			
			std::shared_ptr<WorldMap> getData() const;
			
			ros::Time getTime() {return m_time;}
			
			bool isUpdated() {return m_isUpdated;}
			
			bool setUsed() {m_isUpdated = false;}
			bool setNew() {m_isUpdated = true;}
		};

	}
}


#endif // GUARD_NEIGHBOURS_H