#ifndef SABO_POINTMASS_H
#define SABO_POINTMASS_H

#include <sabo/Vec2.h>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

namespace sabo
{

  class Body;

  class PointMass : public boost::noncopyable
  {
    public:
      typedef boost::shared_ptr< PointMass > Ptr;
    
      PointMass( Body* body, const Vec2& pos, double mass );
      
      Vec2 pos() const;
      Vec2 vel() const;
      double mass() const;
      
      void applyForce( const Vec2& force );
      
      void update( double dt );
      
    private:
      Body* m_body;
      Vec2 m_pos;
      Vec2 m_vel;
      Vec2 m_force;
      double m_mass;
      double m_friction;
      double m_maxSpeed;
  };
  
}

#endif
