////////////////////////////////////////////////////
//	MonitorReceiver.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef MONITOR_RECEIVER_H
#define MONITOR_RECEIVER_H
#pragma once

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>

#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class WorldMap;
		
		class MonitorReceiver
		{
		protected:
			ros::NodeHandle m_node;
		  	ros::Subscriber m_sub;
			
			std::shared_ptr<WorldMap> m_data;
			
			ros::Time m_time;
		protected:
		
			void UpdateMonitorCallBack(const nav_msgs::OccupancyGrid::ConstPtr & msg);
			
		public:
			MonitorReceiver();
			
			~MonitorReceiver();
			
			std::shared_ptr<WorldMap> getData() const;
			
			ros::Time getTime() {return m_time;}
		};

	}
}


#endif // MONITOR_RECEIVER_H