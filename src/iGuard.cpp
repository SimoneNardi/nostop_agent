#include "iGuard.h"
#include "MonitorReceiver.h"
#include "GuardNeighbours.h"
#include "LearningInitializer.h"

#include "area.h"
#include "guard.h"
#include "discretizedArea.h"
#include <LearningWorld.h>

#include "Conversions.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

// 	////////////////////////////////////////////////////
// 	AgentPosition iGuard::getCurrentAgentPosition()
// 	{
// 		Configuration l_config = this->getCurrentConfiguration();
// 		
// 		// TODO identifica posizione dal mondo.
// 		// ...
// 				
// 		AgentPosition l_pos;// = m_guard->getCurrenPosition();
// 		return l_pos;
// 	}
// 	
	////////////////////////////////////////////////////
	nostop_agent::GuardSensorCtrl iGuard::getCameraControl()
	{
	    nostop_agent::GuardSensorCtrl l_ctrl;
	    return l_ctrl;
	}
	
	////////////////////////////////////////////////////
	iGuard::iGuard() 
	: m_learninigWorld(nullptr)
	, m_LGuard(nullptr)
	, m_monitorReceiver(nullptr)
	, m_guardNeighbours(nullptr)
	{
		m_monitorReceiver = std::make_shared<MonitorReceiver>();
		m_guardNeighbours = std::make_shared<GuardNeighbours>();
		m_learningInit = std::make_shared<LearningInitializer>();
		m_learningInit->start();
	}
	
	////////////////////////////////////////////////////
	void iGuard::setCameraCtrl(nostop_agent::GuardSensorCtrl l_ctrl)
	{
		m_currentControl = l_ctrl;
	}

	////////////////////////////////////////////////////
	void iGuard::createLearningAlgorithm( AreaPtr l_area )
	{
		std::shared_ptr<DiscretizedArea> l_space = l_area->discretize();
	
		Lock lock(m_mutex);
// 		geometry_msgs::Point l_geomPoint = this->getCurrentConfigurationPosition();
// 		IDSReal2D l_point = Conversions::Point2IDSReal2D( l_geomPoint );
// 		nostop_agent::GuardSensorCtrl l_ctrl = this->getCameraControl();
// 		CameraPosition l_camera = Conversions::GuardSensorCtrl2CameraPosition( l_ctrl );
// 		AgentPosition l_agentPos(l_point, l_camera);
// 	  
// 		m_LGuard = std::make_shared<Guard>(1, m_learningInit->getID(), l_agentPos, 1, 2);
		
		std::set< std::shared_ptr<Agent> > l_agents; 
		l_agents.insert(m_LGuard);
				
		m_learninigWorld = std::make_shared<LearningWorld>(l_agents, l_space, m_algorithmFLAG);
		
		AgentPosition l_targetPos = m_LGuard->getCurrentPosition(); // TODO: first position calibrated with the learning algorithm.
		// primo comando di movimento per il robot.
		this->setTargetConfigurationToCenterOfSquare( Conversions::IDSReal2D2Point( l_targetPos.getPoint2D() ) );
	}
	
	////////////////////////////////////////////////////
	void iGuard::setTargetConfigurationToCenterOfSquare(geometry_msgs::Point const& target_)
	{
		Lock lock(m_mutex);
		m_targetConfiguration = Configuration(target_);
	}
	
	////////////////////////////////////////////////////
	void iGuard::setGuardPtr(std::shared_ptr<Guard> lGuard_)
	{
	  	Lock lock(m_mutex);
		m_LGuard = lGuard_;
		iAgent::setAgentPtr(m_LGuard);
	  
	}
	
	