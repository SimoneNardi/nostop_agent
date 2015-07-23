#include "iGuard.h"
#include "MonitorReceiver.h"
#include "GuardNeighbours.h"

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
	}