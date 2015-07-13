#include "ros/ros.h"
#include "nostop/PlayerID.h"

#include "agent.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Thief"); 
	
	// get the position:
	
	// get the control:
	
	// get the ID from Simulator:
	ros::NodeHandle l_nodeID;
	ros::ServiceClient l_clientID = l_nodeID.serviceClient<nostop::PlayerID>("ThiefID");
	nostop::PlayerID l_srvID;
	if (l_clientID.call(l_srvID))
	{
		ROS_INFO("Selected ID: %ld", (long int)l_srvID.response.id);
	}
	else
	{
		ROS_ERROR("Failed to call service ThiefID");
		return 1;
	}
	
	return 0;
	
	// create Logical object
	
	// send back to simulator ID, position, control and kind!
	
	//std::shared_ptr<Robotics::GameTheory::Agent> l_agent = std::make_shared<Guard>();
	
	if (argc != 3)
	{
		ROS_INFO("usage: agent_client col row");
		return 1;
	}

	return 0;
}