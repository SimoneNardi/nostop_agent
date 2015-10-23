#include "iGuard.h"
#include "MonitorReceiver.h"
#include "GuardNeighbours.h"

#include "LearningInitializer.h"
#include "LearningProcess.h"

#include "area.h"
#include "guard.h"
#include "discretizedArea.h"
#include "learningWorld.h"

#include "Conversions.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	nostop_agent::GuardSensorCtrl iGuard::getCameraControl()
	{
	    return m_currentControl;
	}
	
	////////////////////////////////////////////////////
	void iGuard::setName(std::string const& name_)
	{
		iAgent::setName(name_);
		
		m_learningInit->setName(name_);
	  	m_learningInit->start();
	}
	
	////////////////////////////////////////////////////
	iGuard::iGuard() 
	: m_learning(nullptr)
	, m_learningInit(nullptr)
	, m_LGuard(nullptr)
	{
		m_learningInit = std::make_shared<LearningInitializer>();
	}
	
	//////////////////////////////////////////////////
	void iGuard::setRobotAlgorithm(std::string alg_)
	{
		if (alg_=="PIPIP")
		    m_algorithmFLAG = Robotics::GameTheory::PIPIP;
		else if (alg_=="PARETO")
		    m_algorithmFLAG = Robotics::GameTheory::PARETO;
		else if (alg_=="CORRELATED")
		    m_algorithmFLAG = Robotics::GameTheory::CORRELATED;
		else 
		    m_algorithmFLAG = Robotics::GameTheory::DISL;
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
		std::set< std::shared_ptr<Agent> > l_agents; 
		l_agents.insert(m_LGuard);
				
		std::shared_ptr<LearningWorld> l_learn = std::make_shared<LearningWorld>(l_agents, l_space, m_algorithmFLAG);
		m_learning = std::make_shared<LearningProcess>(l_learn);
		m_learning->init();
		
		// TODO: first target position is determined by the localizer.
		AgentPosition l_targetPos = m_LGuard->getCurrentPosition(); 
		this->setTargetConfigurationToCenterOfSquare( Conversions::Real2D2Point( l_targetPos.getPoint2D() ) );
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
	
	////////////////////////////////////////////////////
	void iGuard::startLearning()
	{
	  m_learning->start();
	}
		  
	////////////////////////////////////////////////////
	// Perform next step
	void iGuard::forwardOneStep()
	{
	  // go forward in the learninig algorithm:
	  	  
	  // Compute benefit:
	  
	  // Select next target configuration:
	}
	