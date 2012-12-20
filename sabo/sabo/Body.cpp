#include <sabo/Body.h>

namespace sabo
{

  Body::Body( World* world ) :
    m_world( world )
  {
  }
  
  Body::~Body()
  {
  }
  
  void Body::addPointMass( const PointMass::Ptr& pointMass )
  {
    m_pointMasses.push_back( pointMass );
  }
  
  bool Body::removePointMass( const PointMass::Ptr& pointMass )
  {
    for ( std::vector< PointMass::Ptr >::iterator i = m_pointMasses.begin(); i != m_pointMasses.end(); ++i )
    {
      if ( *i == pointMass )
      {
        m_pointMasses.erase( i );
        return true;
      }
    }
    return false;
  }
  
  Body::PointMassIterator Body::pointMassesBegin() const
  {
    return m_pointMasses.begin();
  }
  
  Body::PointMassIterator Body::pointMassesEnd() const
  {
    return m_pointMasses.end();
  }
  
  void Body::addActuator( const Actuator::Ptr& actuator )
  {
    m_actuators.push_back( actuator );
  }
  
  bool Body::removeActuator( const Actuator::Ptr& actuator )
  {
    for ( std::vector< Actuator::Ptr >::iterator i = m_actuators.begin(); i != m_actuators.end(); ++i )
    {
      if ( *i == actuator )
      {
        m_actuators.erase( i );
        return true;
      }
    }
    return false;
  }
  
  Body::ActuatorIterator Body::actuatorsBegin() const
  {
    return m_actuators.begin();
  }
  
  Body::ActuatorIterator Body::actuatorsEnd() const
  {
    return m_actuators.end();
  }
  
  void Body::update( double dt )
  {
    for ( ActuatorIterator i = m_actuators.begin(); i != m_actuators.end(); ++i )
    {
      (*i)->update( dt );
    }
    for ( PointMassIterator i = m_pointMasses.begin(); i != m_pointMasses.end(); ++i )
    {
      (*i)->update( dt );
    }
  }
  
}
