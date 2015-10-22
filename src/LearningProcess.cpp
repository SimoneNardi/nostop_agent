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
  , m_notified(false)
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
  Lock1 lock(m_mutex);
  m_sub = m_node.subscribe<std_msgs::Bool>("/simulator/agent_call", 1, &LearningProcess::AgentCall_CallBack, this);
}

/////////////////////////////////////////////
void LearningProcess::AgentCall_CallBack(const std_msgs::Bool::ConstPtr & msg_)
{
  Lock1 lock(m_mutex);
  m_update = msg_->data;
  m_cond_var.notify_one();
  m_notified = true;
}

/////////////////////////////////////////////
void LearningProcess::run()
{
	ros::Rate loop_rate(1);
	
	Lock1 l_lock(m_mutex);
	
	int count = 0;
	while (ros::ok())
	{
	  
	  while (!m_notified) 
	  {  // loop to avoid spurious wakeups
                m_cond_var.wait(l_lock);
          }
          m_notified = false;
	  
	  if (m_update)
	  {
		// Collect Monitor Data:
		m_learning->updateMonitor(m_monitorReceiver->getData()->getMap());
		
		// Collect Neighbours Data:
		m_learning->updateNeighbours(m_guardNeighbours->getData()->getMap());
		
		// Compute Benefit, Save current action and Select next position:
		m_learning->forwardOneStep();

		m_update = false;
	  }
	  
	  ros::spinOnce();

	  loop_rate.sleep();
	  ++count;
	}
}

/////////////////////////////////////////////
double LearningProcess::getTimeOFMonitor()
{
  return m_monitorReceiver->getTime().toSec();
}

/////////////////////////////////////////////
double LearningProcess::getTimeOFNeighbours()
{
  return m_guardNeighbours->getTime().toSec();
}