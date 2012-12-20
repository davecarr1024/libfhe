#ifndef SABO_SPRING_H
#define SABO_SPRING_H

#include <sabo/Actuator.h>
#include <sabo/PointMass.h>

namespace sabo
{

  class Spring : public Actuator
  {
    public:
      typedef Actuator Base;
      typedef boost::shared_ptr< Spring > Ptr;
      
      Spring( Body* body, const PointMass::Ptr& p1, const PointMass::Ptr& p2 );
      virtual ~Spring();
      
      virtual void update( double dt );
      
    private:
      PointMass::Ptr m_p1;
      PointMass::Ptr m_p2;
      double m_propGain;
      double m_diffGain;
      double m_baseDist;
  };

}

#endif
