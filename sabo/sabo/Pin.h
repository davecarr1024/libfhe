#ifndef SABO_PIN_H
#define SABO_PIN_H

#include <sabo/Actuator.h>
#include <sabo/PointMass.h>

namespace sabo
{

  class Pin : public Actuator
  {
    public:
      typedef Actuator Base;
      typedef boost::shared_ptr< Pin > Ptr;
      
      Pin( Body* body, const PointMass::Ptr& pm );
      virtual ~Pin();
      
      virtual void update( double dt );
    
    private:
      PointMass::Ptr m_pm;
      double m_propGain;
      double m_diffGain;
      Vec2 m_basePos;
  };

}

#endif
