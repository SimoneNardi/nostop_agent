#include "ros/ros.h"
#include "nostop_agent/GuardBenefitData.h"
#include "nostop_agent/GuardNeighboursData.h"
#include "nostop_agent/PlayerIDData.h"
#include "nostop_agent/PlayerConfigurationData.h"
#include "nostop_agent/AreaData.h"

#include "Real2D.h"

#include "AreaCreator.h"
#include "GuardProcess.h"

#include "learningWorld.h"

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
      if (l_node.getParam("robot_name", l_name))
      {
	ROS_INFO("Nome ricevuto: %s", l_name.c_str());
      }
      else
      {
	std::cout << "Enter the name of the robot: ";
	//std::cin >> l_name;
	l_name="red_blue";
	ROS_ERROR("Nome non ricevuto: %s", l_name.c_str());
      }
	
      Robotics::GameTheory::GuardProcess l_guard(l_name);
       
      ////////////////////////////////////////
      // Identify sensor localizer:
      std::string l_str;
      if (l_node.getParam("localizer", l_str))
      {
	l_guard.setKinectLocalizer();
	ROS_INFO("Kinect Localizer.");
      }
      else
      {
	l_guard.setSimulatorLocalizer();
	ROS_INFO("Simulator Localizer.");
      }
            
      ////////////////////////////////////////
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

      ///////////////////////////////////////////////
      // Build the area
      Robotics::GameTheory::AreaPtr l_area = nullptr;
      
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
      
      l_guard.setAreaForInitialization(l_area);

      while (!l_guard.isReady())
	ros::spinOnce();
	
      l_guard.createLearningAlgorithm();
	
      l_guard.start();
      
      /////////////////////////////////////////////////
      // WAIT FOR ROS MESSAGES
      ros::spin();
      
      ROS_INFO("Ending agent %s.", l_name.c_str());

      return 0;
}