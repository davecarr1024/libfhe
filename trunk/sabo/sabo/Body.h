#ifndef SABO_BODY_H
#define SABO_BODY_H

#include <sabo/Actuator.h>
#include <sabo/PointMass.h>
#include <vector>

namespace sabo
{

  class World;

  class Body : public boost::noncopyable
  {
    public:
      Body( World* world );
      virtual ~Body();
      
      typedef std::vector< PointMass::Ptr >::const_iterator PointMassIterator;
      void addPointMass( const PointMass::Ptr& pointMass );
      bool removePointMass( const PointMass::Ptr& pointMass );
      PointMassIterator pointMassesBegin() const;
      PointMassIterator pointMassesEnd() const;
      
      typedef std::vector< Actuator::Ptr >::const_iterator ActuatorIterator;
      void addActuator( const Actuator::Ptr& actuator );
      bool removeActuator( const Actuator::Ptr& actuator );
      ActuatorIterator actuatorsBegin() const;
      ActuatorIterator actuatorsEnd() const;
      
      void update( double dt );
      
    private:
      World* m_world;
      std::vector< PointMass::Ptr > m_pointMasses;
      std::vector< Actuator::Ptr > m_actuators;
  };

}

#endif
