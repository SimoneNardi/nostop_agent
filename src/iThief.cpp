#include "iThief.h"

using namespace Robotics;
using namespace Robotics::GameTheory;
using namespace std;

////////////////////////////////////////////////////
iThief::iThief() 
: iAgent()
, m_learninigWorld(nullptr)
, m_LThief(nullptr)
{}

////////////////////////////////////////////////////
void iThief::setName(std::string const& name_)
{
  Lock lock(m_mutex);
  iAgent::setName(name_);
}