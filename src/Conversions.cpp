#include "Conversions.h"

using namespace Robotics;
using namespace Robotics::GameTheory;

///
IDSReal2D Conversions::Point2IDSReal2D(geometry_msgs::Point const& point_)
{
  IDSReal2D l_res(point_.x, point_.y);
  return l_res;
}

///
geometry_msgs::Point Conversions::IDSReal2D2Point(IDSReal2D const& point_)
{
  geometry_msgs::Point l_res;
  l_res.x = point_.v[0];
  l_res.y = point_.v[1];
  return l_res;
}

///
nostop_agent::GuardSensorCtrl Conversions::CameraPosition2GuardSensorCtrl(CameraPosition const& ctrl_)
{
   nostop_agent::GuardSensorCtrl l_camera;
   l_camera.max_radius = ctrl_.getFarRadius(); 
   l_camera.max_radius = ctrl_.getNearRadius();
   l_camera.heading = ctrl_.getOrientation();
   l_camera.fov = ctrl_.getAngleOfView();
  return l_camera;
}

///
CameraPosition Conversions::GuardSensorCtrl2CameraPosition(nostop_agent::GuardSensorCtrl const& ctrl_)
{
  CameraPosition l_camera( ctrl_.max_radius, ctrl_.min_radius, ctrl_.heading, ctrl_.fov );
  return l_camera;
}
