#include "LearningProcess.h"

#include "std_msgs/Bool.h"

#include "ros/ros.h"

#include "iAgent.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
LearningProcess::LearningProcess(int id_) 
  : ThreadBase()
  , m_id(id_)
  , m_update(false)
{}

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
	ros::Rate loop_rate(5);
	
	int count = 0;
	while (ros::ok() && m_update)
	{
	  
	  m_mutex.lock();
	    m_update = false;
	  m_mutex.unlock();
	  
	  ros::spinOnce();

	  loop_rate.sleep();
	  ++count;
	}
}