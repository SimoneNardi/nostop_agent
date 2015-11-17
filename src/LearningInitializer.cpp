#include "LearningInitializer.h"

#include "iAgent.h"

#include "nostop_agent/PlayerIDData.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

  ///////////////////////////////////////////////
  LearningInitializer::LearningInitializer(std::string const& name_) 
  : ThreadBase()
  , m_id(-1) 
  , m_name(name_)
  , m_area(nullptr)
  {}
  
  ///////////////////////////////////////////////
  void LearningInitializer::setName(std::string const& name_)
  {
    m_name = name_;
  }
  
  ///////////////////////////////////////////////
  LearningInitializer::~LearningInitializer()
  {}

  ///////////////////////////////////////////////
  void LearningInitializer::run()
  {
    ros::Rate loop_rate(5);
    
    ROS_INFO("Waiting for Monitor Simulator.");
    
    int count = 0;
    while (ros::ok() && m_id < 0)
    {
      ///////////////////////////////////////////////
      // get the ID from simulator
      ros::NodeHandle l_nodeID;
      ros::ServiceClient l_clientID = l_nodeID.serviceClient<nostop_agent::PlayerIDData>("/simulator/guard/id");
      nostop_agent::PlayerIDData l_srvID;
      l_srvID.request.name = m_name;
      if (l_clientID.call(l_srvID))
      {
	      ROS_INFO("Selected ID: %ld", (long int)l_srvID.response.id);
	      Lock l_lock(m_mutex);
	      m_id = (long int)l_srvID.response.id;
      }
      else
      {
	ROS_ERROR("Failed to call service Guard ID");
      }
      
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }
  }
  
  ///////////////////////////////////////////////
  bool LearningInitializer::isInitialized() const
  {
    return !(m_id < 0);
  }
  
  ///////////////////////////////////////////////
  int LearningInitializer::getID() const
  {
    return m_id;
  }
  
  ///////////////////////////////////////////////
  void LearningInitializer::setSpace(std::shared_ptr<Area> area_)
  {
    m_area = area_;
  }

  ///////////////////////////////////////////////
  std::shared_ptr<Area> LearningInitializer::getSpace() const
  {
    return m_area;
  }
  