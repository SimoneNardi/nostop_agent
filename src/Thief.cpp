#include "ros/ros.h"
#include "nostop_agent/PlayerIDData.h"

#include "ThiefProcess.h"

#include "agent.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Thief"); 
	
	std::string l_name = "Thief";
	
	// get the ID from Simulator:
	ros::NodeHandle l_nodeID;
	ros::ServiceClient l_clientID = l_nodeID.serviceClient<nostop_agent::PlayerIDData>("/simulator/thief/id");
	nostop_agent::PlayerIDData l_srvID;
	l_srvID.request.name = l_name;
	if (l_clientID.call(l_srvID))
	{
		ROS_INFO("Selected ID: %ld", (long int)l_srvID.response.id);
	}
	else
	{
		ROS_ERROR("Failed to call service ThiefID");
		//return 1;
		
		l_srvID.response.id = 1;
	}
	
	Robotics::GameTheory::ThiefProcess l_thief(l_name, l_srvID.response.id);
	
	l_thief.setSimulatorLocalizer();
	ROS_INFO("Thief Simulator Localizer.");
	
	while (!l_thief.isReady())
	  ros::spinOnce();
	
	l_thief.start();

	ros::spin();
	
	return 0;
}