#include "iGuard.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	AgentPosition iGuard::getCurrentAgentPosition()
	{
		Configuration l_config = this->getCurrentConfiguration();
		
		// TODO identifica posizione dal mondo.
		// ...
				
		AgentPosition l_pos;// = m_guard->getCurrenPosition();
		return l_pos;
	}