#include "ros/ros.h"

#include "nostop_agent/GuardBenefitData.h"
#include "nostop_agent/GuardNeighboursData.h"
//#include "nostop_agent/PlayerIDData.h"
#include "nostop_agent/PlayerConfigurationData.h"

#include "nostop_area/AreaData.h"

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
	  l_name="red_blue";
	  ROS_ERROR("Robot name NOT received: %s", l_name.c_str());
      }
	
      Robotics::GameTheory::GuardProcess l_guard(l_name);
       
      ////////////////////////////////////////
      // Identify sensor localizer:
      if (l_node.searchParam("simulate", temp_name))
      {
	int l_simulate;
	l_node.getParam(temp_name,l_simulate);
	if(l_simulate == 1)
	{
	  l_guard.setSimulatorLocalizer();
	  ROS_INFO("Simulator Localizer");
	}
	else
	{
	  if( l_guard.setKinectLocalizer() )
	  {
	    l_guard.setSimulatorLocalizer();
	    ROS_INFO("Simulator Localizer");
	  }
	  else
	    ROS_INFO("Kinect Localizer");
	}
      }
      else
      {
	ROS_ERROR("Localizer not received: %s", l_name.c_str());
      }
            
      ////////////////////////////////////////
      // Identify Robot Algorithm:
      std::string l_str;
      if (l_node.searchParam("learning_name", temp_name))
      {
	l_node.getParam(temp_name,l_str);
      	l_guard.setRobotAlgorithm(l_str);
	ROS_INFO("Learning received: %s", l_str.c_str());
      }
      else
      {
	l_str = "DISL";
	l_guard.setRobotAlgorithm(l_str);
	ROS_ERROR("Learning not received: %s", l_str.c_str());
      }
         
      ////////////////////////////////////////
      // Identify Robot Period:
      int l_period = 1;
      if (l_node.searchParam("period", temp_name))
      {
	l_node.getParam(temp_name,l_str);
	l_period = atoi(l_str.c_str());
	l_guard.setPeriod(l_period);
	ROS_INFO("Period received: %d", l_period);
      }
      else
      {
	l_guard.setPeriod(l_period);
	ROS_ERROR("Period not received: %d", l_period);
      }

      ///////////////////////////////////////////////
      // Build the area
      Robotics::GameTheory::AreaPtr l_area = nullptr;
      
      ros::NodeHandle l_nodeArea;
      ros::ServiceClient l_clientArea = l_nodeArea.serviceClient<nostop_area::AreaData>("/AreaInitializer");
      nostop_area::AreaData l_srvArea;
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
      ROS_INFO("Area Assigned!");

      ros::Rate loop_rate(5);
      while (!l_guard.isReady())
      {
	ros::spinOnce();
	loop_rate.sleep();
      }
      ROS_INFO("Guard Ready!");
      
      l_guard.createLearningAlgorithm();

      l_guard.start();
      ROS_INFO("Guard Started!");
      
      /////////////////////////////////////////////////
      // WAIT FOR ROS MESSAGES
      ros::spin();
      
      ROS_INFO("Ending agent %s.", l_name.c_str());

      return 0;
}