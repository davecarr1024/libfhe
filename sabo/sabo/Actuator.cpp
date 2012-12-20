#include <sabo/Actuator.h>

namespace sabo
{
  
  Actuator::Actuator( Body* body ) :
    m_body( body )
  {
  }
  
  Actuator::~Actuator()
  {
  }
  
  Body* Actuator::body() const
  {
    return m_body;
  }

}
