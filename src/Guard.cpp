#include "ros/ros.h"
#include "nostop_agent/GuardBenefitData.h"
#include "nostop_agent/GuardNeighboursData.h"
#include "nostop_agent/PlayerIDData.h"
#include "nostop_agent/PlayerConfigurationData.h"
#include "nostop_agent/AreaData.h"

#include "IDSReal2D.h"

#include "AgentAreaCreator.h"
#include "GuardInterface.h"

#include "LearningWorld.h"

#include <cstdlib>

#include "agent.h"

std::shared_ptr<Robotics::GameTheory::LearningWorld> g_coverage = nullptr;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Guard"); 
	
	Robotics::GameTheory::AreaPtr l_area = nullptr;
	
	///////////////////////////////////////////////
	// Build the area
	ros::NodeHandle l_nodeArea;
	ros::ServiceClient l_clientArea = l_nodeArea.serviceClient<nostop_agent::AreaData>("AreaInitializer");
	nostop_agent::AreaData l_srvArea;
	if (l_clientArea.call(l_srvArea))
	{
		ROS_INFO("Area description received");
		
		// creazione dell'area:
		Robotics::GameTheory::AgentAreaCreator l_areaCreator(l_srvArea.response.external, l_srvArea.response.internal);
		l_area = l_areaCreator.getArea();
	}
	else
	{
		ROS_ERROR("Failed to call service AreaInitializer");
		Robotics::GameTheory::AgentAreaCreator l_areaCreator;
		l_area = l_areaCreator.getArea();
	}
	
	///////////////////////////////////////////////
	// get the ID from simulator
	ros::NodeHandle l_nodeID;
	ros::ServiceClient l_clientID = l_nodeID.serviceClient<nostop_agent::PlayerIDData>("GuardID");
	nostop_agent::PlayerIDData l_srvID;
	if (l_clientID.call(l_srvID))
	{
		ROS_INFO("Selected ID: %ld", (long int)l_srvID.response.id);
	}
	else
	{
		ROS_ERROR("Failed to call service GuardID");
		return 1;
	}
	
	///////////////////////////////////////////////
	// Get the agent configuration:
	ros::NodeHandle l_nodePos;
	ros::ServiceClient l_clientPos = l_nodePos.serviceClient<nostop_agent::PlayerConfigurationData>("GuardInitialPosition");
	nostop_agent::PlayerConfigurationData l_srvPos;
	l_srvPos.request.id = l_srvID.response.id;
	if (l_clientPos.call(l_srvPos))
	{
		ROS_INFO("Initial configuration for Player %ld is: %ld, %ld, %ld", (long int)l_srvID.response.id, (long int)l_srvPos.response.x, (long int)l_srvPos.response.y, (long int)l_srvPos.response.heading);
	}
	else
	{
		ROS_ERROR("Failed to call service GuardInitialPosition");
		l_srvPos.response.x = 5;
		l_srvPos.response.y = 5;
	}
	
	// publish agent configuration to simulator
	Robotics::GameTheory::CameraPosition l_camera(l_area->getDistance() / 10.);
	IDSReal2D l_position(l_srvPos.response.x, l_srvPos.response.y);
	std::shared_ptr<Robotics::GameTheory::GuardInterface> l_agent = std::make_shared<Robotics::GameTheory::GuardInterface>( l_srvID.response.id, Robotics::GameTheory::AgentPosition(l_position, l_camera) );
	
	std::shared_ptr<Robotics::GameTheory::Agent> l_learningAgent = l_agent->getAgent();
	g_coverage = std::make_shared<Robotics::GameTheory::LearningWorld>(l_learningAgent, l_area->discretize(), Robotics::GameTheory::DISL);
	
	return 0;
	
	
	// create Logical object
	
	// send back to simulator ID, position, control and kind!
	
	//std::shared_ptr<Robotics::GameTheory::Agent> l_agent = std::make_shared<Guard>();
	
	if (argc != 3)
	{
		ROS_INFO("usage: nostop_agent col row");
		return 1;
	}

	// Benefit
	ros::NodeHandle l_nodeBenefit;
	ros::ServiceClient clientBenefit = l_nodeBenefit.serviceClient<nostop_agent::GuardBenefitData>("GuardBenefit");
	nostop_agent::GuardBenefitData srvBenefit;
	srvBenefit.request.col = atoll(argv[1]);
	srvBenefit.request.row = atoll(argv[2]);
	if (clientBenefit.call(srvBenefit))
	{
		ROS_INFO("Benefit: %ld", (long int)srvBenefit.response.benefit);
	}
	else
	{
		ROS_ERROR("Failed to call service GuardBenefit");
		return 1;
	}

	// Number of neighbors
	ros::NodeHandle l_nodeNeighbours;
	ros::ServiceClient clientNeighbours = l_nodeNeighbours.serviceClient<nostop_agent::GuardNeighboursData>("GuardNeighbours");
	nostop_agent::GuardNeighboursData srvNeighbours;
	srvNeighbours.request.col = atoll(argv[1]);
	srvNeighbours.request.row = atoll(argv[2]);
	if (clientNeighbours.call(srvNeighbours))
	{
		ROS_INFO("Neighbours: %ld", (long int)srvNeighbours.response.neighbors);
	}
	else
	{
		ROS_ERROR("Failed to call service GuardNeighbours");
		return 1;
	}



	return 0;
}