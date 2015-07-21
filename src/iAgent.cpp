#include "iAgent.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	bool operator==(const Configuration& lhs, const Configuration& rhs)
	{
	  geometry_msgs::Quaternion lhs_orientation = lhs.getOrientation();
	  geometry_msgs::Point lhs_position = lhs.getPosition();
	  
	  geometry_msgs::Quaternion rhs_orientation = rhs.getOrientation();
	  geometry_msgs::Point rhs_position = rhs.getPosition();
	  
	  return lhs_orientation==rhs_orientation && lhs_position==rhs_position;
	}

	////////////////////////////////////////////////////
	bool operator!=(const Configuration& lhs, const Configuration& rhs)
	{
	  return !(lhs==rhs);
	}

	////////////////////////////////////////////////////
	void iAgent::setCurrentOrientation(geometry_msgs::Quaternion & orientation_) 
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setOrientation(orientation_);
	}
	
	////////////////////////////////////////////////////
	void setCurrentPosition(geometry_msgs::Point & point_)
	{
	  Lock lock(m_mutex);
	  m_currentConfiguration.setPosition(point_);
	}

	////////////////////////////////////////////////////
	AgentPosition getCurrentPosition()
	{
	  //TODO ... calcolo dell'agentPosition.
	  AgentPosition l_pos;
	  return l_pos;
	}
	
	////////////////////////////////////////////////////
	AgentPosition  iAgent::computeAgentPosition(geometry_msgs::Point & point_)
	{
	  Lock lock(m_mutex);
	  AgentPosition l_pos;
	  return l_pos;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setAgentPtr(std::shared_ptr<Agent> agent_)
	{
	  Lock lock(m_mutex);
	  m_agent = agent_;
	}
	
	////////////////////////////////////////////////////
	void iAgent::setLocalizer(ColorName back_, ColorName front_)
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<KinectLocalizer>(back_, front_);
	}
	
	////////////////////////////////////////////////////
	void iAgent::setLocalizer(std::string name_)
	{
	  Lock lock(m_mutex);
	  m_localizer = std::make_shared<SimulatorLocalizer>(name_);
	}
	
	////////////////////////////////////////////////////
	double iAgent::getCurrentAngularSpeed()
	{
	  return m_agent->getCurrentRotation();
	}
	
	////////////////////////////////////////////////////
	double iAgent::getCurrentLinearSpeed()
	{
	  return m_agent->getCurrentSpeed();
	}