#include "AreaCreator.h"

#include "area.h"
#include "structuredArea.h"

#include "nostop_area/Shape.h"

using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////////
AreaCreator::AreaCreator() 
	: m_external()
	, m_internal()
{
	// Collect point from rviz or read configuration file if not available
	std::vector<Real2D> l_points;

	l_points.push_back(Real2D (-10.,-10.) );
	l_points.push_back(Real2D (10.,-10.) );
	l_points.push_back(Real2D (10.,10.) );
	l_points.push_back(Real2D (-10.,10.) );

	m_external = l_points;
}

/////////////////////////////////////////////////
AreaCreator::AreaCreator(nostop_area::Shape external_, std::vector<nostop_area::Shape> internal_) 
	: m_external()
	, m_internal()
{
	// Collect point from rviz or read configuration file if not available
	
	std::vector<Real2D> l_points;
	for(size_t i=0; i < external_.vertex.size()/2; i++)
	{
	  l_points.push_back( Real2D (external_.vertex[2*i+0],external_.vertex[2*i+1]) );
	}
	m_external = l_points;
	
	for(size_t j=0; j < internal_.size(); ++j)
	{
	  std::vector<Real2D> l_obs;
	  for(size_t i=0; i < internal_[j].vertex.size()/2; i++)
	  {
	      l_obs.push_back( Real2D (external_.vertex[2*i+0],external_.vertex[2*i+1]) );
	  }
	  if (l_obs.size() > 2)
	    m_internal.push_back(l_obs);
	}
}

/////////////////////////////////////////////////
AreaPtr AreaCreator::getArea() const
{
	AreaPtr l_area = std::make_shared<StructuredArea>(m_external, m_internal);
	return l_area;
}