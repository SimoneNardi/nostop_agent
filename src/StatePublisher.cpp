#include "StatePublisher.h"

#include "nostop_agent/GuardState.h"
#include "nostop_agent/GuardSensorCtrl.h"

#include "ros/ros.h"

#include "iGuard.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
StatePublisher::StatePublisher(std::shared_ptr<iAgent> agent_) 
: ThreadBase()
{}

/////////////////////////////////////////////
StatePublisher::~StatePublisher()
{}