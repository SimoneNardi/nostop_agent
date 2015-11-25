#include "iGuard.h"
#include "MonitorReceiver.h"
#include "GuardNeighbours.h"

#include "LearningInitializer.h"
#include "LearningProcess.h"

#include "area.h"
#include "guard.h"
#include "discretizedArea.h"
#include "learningWorld.h"

#include "Box.h"

#include "nostop_agent/PlayerNotifyStatus.h"

#include "Conversions.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

	////////////////////////////////////////////////////
	nostop_agent::GuardSensorCtrl iGuard::getCameraControl()
	{
	  Lock lock(m_mutex);
	    return m_currentControl;
	}
	
	////////////////////////////////////////////////////
	void iGuard::setName(std::string const& name_)
	{
	  Lock lock(m_mutex);
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
	  Lock lock(m_mutex);
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
	  Lock lock(m_mutex);
		m_currentControl = l_ctrl;
	}
	
	////////////////////////////////////////////////////
	void iGuard::setCameraCtrl(CameraPosition const& l_ctrl)
	{
	    Lock lock(m_mutex);
		m_currentControl.fov = l_ctrl.getAngleOfView();
		m_currentControl.heading = l_ctrl.getOrientation();
		m_currentControl.max_radius = l_ctrl.getFarRadius();
		m_currentControl.min_radius = l_ctrl.getNearRadius();
	}

	////////////////////////////////////////////////////
	void iGuard::createLearningAlgorithm(  )
	{
		AreaPtr l_area = m_learningInit->getSpace();
		std::shared_ptr<DiscretizedArea> l_space = l_area->discretize();
		
		Lock lock(m_mutex);
		m_square_side = (l_space->getXStep() + l_space->getYStep()) /2;
		
		std::set< std::shared_ptr<Agent> > l_agents; 
		l_agents.insert(m_LGuard);
				
		std::shared_ptr<LearningWorld> l_learn = std::make_shared<LearningWorld>(l_agents, l_space, m_algorithmFLAG);
		m_learning = std::make_shared<LearningProcess>(l_learn, m_name);
		m_learning->init();
		
		// first target position is determined by the localizer.
		AgentPosition l_targetPos = m_LGuard->getCurrentPosition();
		
		CameraPosition l_cameraCtrl = l_targetPos.getCameraControl();
		this->setCameraCtrl(l_cameraCtrl);
		
		//this->setTargetConfigurationToCenterOfSquare( Conversions::Real2D2Point( l_targetPos.getPoint2D() ) );
		
		SquarePtr l_square = l_space->getSquare(l_targetPos.getPoint2D());
		Real3D l_target3D = l_square->getBoundingBox().center();
		Real2D l_target2D ( l_target3D[0], l_target3D[1] );
		this->setTargetConfigurationToCenterOfSquare( Conversions::Real2D2Point( l_target2D ) );
	}
	
	////////////////////////////////////////////////////
	void iGuard::setTargetConfigurationToCenterOfSquare(geometry_msgs::Point const& target_)
	{
		Lock lock(m_mutex);
		this->setTargetConfiguration( Configuration(target_) );
		m_LGuard->setNextPosition( Real2D(target_.x, target_.y) );
	}
	
	////////////////////////////////////////////////////
	void iGuard::setGuardPtr(std::shared_ptr<Guard> lGuard_)
	{
	  	Lock lock(m_mutex);
		m_LGuard = lGuard_;
		
		std::string  l_name = "/publisher/status";
		m_notifyStatus = m_node.serviceClient<nostop_agent::PlayerNotifyStatus>(l_name.c_str());
		
		iAgent::setAgentPtr(m_LGuard);
	}
	
	////////////////////////////////////////////////////
	void iGuard::startLearning()
	{
	  Lock lock(m_mutex);
	  m_learning->start();
	}	
	
	//////////////////////////////////////////////////
	// send a broadcast message of unemployed agents
	bool iGuard::notifyStatus()
	{
	  nostop_agent::PlayerNotifyStatus l_srv;
	  l_srv.request.id = this->getID();
	  l_srv.request.status = this->getStatus();
	  
	  if(!m_notifyStatus.exists())
	    m_notifyStatus.waitForExistence();
	  	  
	  if ( !m_notifyStatus.exists() || !m_notifyStatus.call(l_srv) )
	  {
	      ROS_ERROR("Failed to call service Notify Status");
	      return false;
	  }
	  else
	  {
	      ROS_DEBUG("Notify Status of Agent %d: Status %d", l_srv.request.id, l_srv.request.status);
	  }

	  return true;
	}