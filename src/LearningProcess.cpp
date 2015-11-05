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
LearningProcess::LearningProcess(std::shared_ptr<LearningWorld> learning_, std::string const& name_) 
  : ThreadBase()
  , m_learning(learning_)
  , m_update(false)
  , m_monitorReceiver(nullptr)
  , m_guardNeighbours(nullptr)
  , m_notified(false)
  , m_name(name_)
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
  m_subAgentCall = m_node.subscribe<std_msgs::Bool>("/simulator/agent_call", 1, &LearningProcess::AgentCall_CallBack, this);
  
  std::string l_name = "/";
  l_name += m_name;
  l_name += "/update";
  m_pubForward = m_node.advertise<std_msgs::Bool>(l_name.c_str(), 1);
}

/////////////////////////////////////////////
void LearningProcess::AgentCall_CallBack(const std_msgs::Bool::ConstPtr & msg_)
{
  Lock1 lock(m_mutex);
  m_update = msg_->data;
  m_notified = true;
  m_cond_var.notify_all();
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
		if(m_monitorReceiver->isUpdated() && m_guardNeighbours->isUpdated())
		{
		  // Collect Monitor Data:
		  m_learning->updateMonitor( m_monitorReceiver->getData()->getMap() );
		  m_monitorReceiver->setUsed();
		
		  // Collect Neighbours Data:
		  m_learning->updateNeighbours( m_guardNeighbours->getData()->getMap() );
		  m_guardNeighbours->setUsed();
		
		  // Compute Benefit, Save current action and Select next position:
		  m_learning->forwardOneStep();
		  
		  std_msgs::Bool l_msg;
		  l_msg.data = true;
		  m_pubForward.publish(l_msg);
		  
		  ROS_DEBUG("Learning Process Update!\n");
		  
		  m_update = false;
		}
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