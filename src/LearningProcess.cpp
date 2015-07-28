#include "LearningProcess.h"
#include "MonitorReceiver.h"
#include "GuardNeighbours.h"
#include "WorldMap.h"

#include "std_msgs/Bool.h"

#include "ros/ros.h"

#include "learningWorld.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
LearningProcess::LearningProcess(std::shared_ptr<LearningWorld> learning_) 
  : ThreadBase()
  , m_learning(learning_)
  , m_update(false)
  , m_monitorReceiver(nullptr)
  , m_guardNeighbours(nullptr)
{
	m_monitorReceiver = std::make_shared<MonitorReceiver>();
	m_guardNeighbours = std::make_shared<GuardNeighbours>();
}

/////////////////////////////////////////////
LearningProcess::~LearningProcess()
{}

/////////////////////////////////////////////
void LearningProcess::init()
{
  Lock lock(m_mutex);
	m_sub = m_node.subscribe<std_msgs::Bool>("AgentCall", 1, &LearningProcess::AgentCall_CallBack, this);
}

void LearningProcess::AgentCall_CallBack(const std_msgs::Bool::ConstPtr & msg_)
{
  Lock lock(m_mutex);
  m_update = msg_->data;
}

/////////////////////////////////////////////
void LearningProcess::run()
{
	ros::Rate loop_rate(1);
	
	int count = 0;
	while (ros::ok() && m_update)
	{
	  // Collect Monitor Data:
	  m_learning->updateMonitor(m_monitorReceiver->getData()->getMap());
	  
	  // Collect Neighbours Data:
	  m_learning->updateNeighbours(m_guardNeighbours->getData()->getMap());
	  
	  // Compute Benefit, Save current action and Select next position:
	  m_learning->forwardOneStep();
	  	  
	  m_mutex.lock();
	    m_update = false;
	  m_mutex.unlock();
	  
	  ros::spinOnce();

	  loop_rate.sleep();
	  ++count;
	}
}