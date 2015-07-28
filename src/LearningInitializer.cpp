#include "LearningInitializer.h"

#include "iAgent.h"

#include "nostop_agent/PlayerIDData.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

  ///////////////////////////////////////////////
  LearningInitializer::LearningInitializer() 
  : ThreadBase()
  , m_id(-1) 
  {}
  
  ///////////////////////////////////////////////
  LearningInitializer::~LearningInitializer()
  {}

  ///////////////////////////////////////////////
  void LearningInitializer::run()
  {
    ros::Rate loop_rate(1);
    
    ROS_INFO("Waiting for Monitor Simulator.");
    
    int count = 0;
    while (ros::ok() && m_id < 0)
    {
      ///////////////////////////////////////////////
      // get the ID from simulator
      ros::NodeHandle l_nodeID;
      ros::ServiceClient l_clientID = l_nodeID.serviceClient<nostop_agent::PlayerIDData>("GuardID");
      nostop_agent::PlayerIDData l_srvID;
      if (l_clientID.call(l_srvID))
      {
	      ROS_INFO("Selected ID: %ld", (long int)l_srvID.response.id);
	      Lock l_lock(m_mutex);
	      m_id = (long int)l_srvID.response.id;
      }
      
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }
  }
  
  ///////////////////////////////////////////////
  bool LearningInitializer::isLearningInitialized() const
  {
    return !(m_id < 0);
  }
  
  ///////////////////////////////////////////////
  int LearningInitializer::getID() const
  {
    return m_id;
  }