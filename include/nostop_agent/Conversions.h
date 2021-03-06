////////////////////////////////////////////////////
//	Conversions.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef CONVERSIONS_H
#define CONVERSIONS_H
#pragma once

#include "Real2D.h"

#include "nostop_agent/GuardSensorCtrl.h"

#include "geometry_msgs/Point.h"

#include "agent.h"

namespace Robotics
{
	namespace GameTheory
	{
		class Conversions
		{
		public:
			static Real2D Point2Real2D(geometry_msgs::Point const&);
			static geometry_msgs::Point Real2D2Point(Real2D const&);
		      
			static CameraPosition GuardSensorCtrl2CameraPosition(nostop_agent::GuardSensorCtrl const& ctrl_);
			static nostop_agent::GuardSensorCtrl CameraPosition2GuardSensorCtrl(CameraPosition const& ctrl_);
		};
	}
}


#endif // CONVERSIONS_H

