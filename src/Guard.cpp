#include "ros/ros.h"
#include "nostop_agent/GuardBenefitData.h"
#include "nostop_agent/GuardNeighboursData.h"
#include "nostop_agent/PlayerIDData.h"
#include "nostop_agent/PlayerConfigurationData.h"
#include "nostop_agent/AreaData.h"

#include "IDSReal2D.h"

#include "AreaCreator.h"
#include "GuardProcess.h"

#include "LearningWorld.h"

#include <cstdlib>

#include "agent.h"
#include "iGuard.h"

		    
int main(int argc, char **argv)
{
      ros::init(argc, argv, "Guard");
                  
      // Info From Launch File
      
      // Identify robot name:
      std::string l_name;
      ros::NodeHandle l_node("~");
      //ros::NodeHandle l_node;
      if (l_node.getParam("robot_name", l_name))
      {
	ROS_INFO("Nome ricevuto: %s", l_name.c_str());
      }
      else
      {
	l_name="red_blue";
	ROS_ERROR("Nome non ricevuto: %s", l_name.c_str());
      }
	
      Robotics::GameTheory::GuardProcess l_guard(l_name);
        
      std::string l_str;
      // Identify Robot Algorithm:
      if (l_node.getParam("learning_name", l_str))
      {
	l_guard.setRobotAlgorithm(l_str);
	ROS_INFO("Learning ricevuto: %s", l_str.c_str());
      }
      else
      {
	l_guard.setRobotAlgorithm("DISL");
	ROS_ERROR("Learning non ricevuto: %s", l_str.c_str());
      }
      
      // Info From Kinect Sensor (Initial Area and Localization)
      
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
	      Robotics::GameTheory::AreaCreator l_areaCreator(l_srvArea.response.external, l_srvArea.response.internal);
	      l_area = l_areaCreator.getArea();
      }
      else
      {
	      ROS_ERROR("Failed to call service AreaInitializer");
	      Robotics::GameTheory::AreaCreator l_areaCreator;
	      l_area = l_areaCreator.getArea();
      }
      	
// 	///////////////////////////////////////////////
// 	// Get the agent configuration:
// 	ros::NodeHandle l_nodePos;
// 	ros::ServiceClient l_clientPos = l_nodePos.serviceClient<nostop_agent::PlayerConfigurationData>("GuardInitialPosition");
// 	nostop_agent::PlayerConfigurationData l_srvPos;
// 	l_srvPos.request.id = l_srvID.response.id;
// 	if (l_clientPos.call(l_srvPos))
// 	{
// 		ROS_INFO("Initial configuration for Player %ld is: %ld, %ld, %ld", (long int)l_srvID.response.id, (long int)l_srvPos.response.x, (long int)l_srvPos.response.y, (long int)l_srvPos.response.heading);
// 	}
// 	else
// 	{
// 		ROS_ERROR("Failed to call service GuardInitialPosition");
// 		l_srvPos.response.x = 5;
// 		l_srvPos.response.y = 5;
// 	}

	while (!l_guard.isReady())
	  ros::spinOnce();
	
	nostop_agent::GuardSensorCtrl l_camera;
	l_camera.max_radius = l_area->getDistance() / 10.;
	
	l_guard.setCamera(l_camera);

	
	// publish agent configuration to simulator
	
	
	// Info From Monitor Sensor (Learning Benefit and Neighbours)
	
	
// 	l_guard.setID(l_srvID.response.id);
	
	//std::shared_ptr<Robotics::GameTheory::Agent> l_learningAgent = l_agent->getAgent();
	//g_coverage = std::make_shared<Robotics::GameTheory::LearningWorld>(l_learningAgent, l_area->discretize(), Robotics::GameTheory::DISL);
	
	// l'agente deve poter scegliere se compiere un'azione oppure se proseguire la traiettoria assegnata, 
	// inoltre deve poter inviare un messaggio al simulatore ogni volta che finisce di compiere l'azione
	
	/////////////////////////////////////////////////
	// WAIT FOR ROS MESSAGES
	ros::spin();

	ROS_INFO("Ending agent %s.", l_name.c_str());


	return 0;
}