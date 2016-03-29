#include "ros/ros.h"
#include "nostop_agent/PlayerIDData.h"

#include "ThiefProcess.h"

#include "agent.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Thief"); 
	
	std::string l_name;
	ros::NodeHandle l_node("~");
	
	std::string temp_name;
	if ( l_node.searchParam("robot_name", temp_name) )
	{
	    l_node.getParam(temp_name,l_name);
	    ROS_INFO("Robot name received: %s", l_name.c_str());
	}
	else
	{
	    std::cout << "Enter the name of the robot: ";
	    //std::cin >> l_name;
	    l_name="thief";
	    ROS_ERROR("Robot name not received: %s", l_name.c_str());
	}
	
	// get the ID from Simulator:
	ros::ServiceClient l_clientID = l_node.serviceClient<nostop_agent::PlayerIDData>("/simulator/thief/id");
	nostop_agent::PlayerIDData l_srvID;
	l_srvID.request.name = l_name;
	l_srvID.request.type = 2;
	if (l_clientID.call(l_srvID))
	{
		ROS_INFO("Selected ID: %ld", (long int)l_srvID.response.id);
	}
	else
	{
		ROS_ERROR("Failed to call service Thief ID");
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