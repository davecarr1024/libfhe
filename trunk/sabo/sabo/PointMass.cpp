#include <sabo/PointMass.h>

namespace sabo
{

  PointMass::PointMass( Body* body, const Vec2& pos, double mass ) :
    m_body( body ),
    m_pos( pos ),
    m_mass( mass ),
    m_friction( 0.25 ),
    m_maxSpeed( 10 )
  {
  }
  
  Vec2 PointMass::pos() const
  {
    return m_pos;
  }
  
  Vec2 PointMass::vel() const
  {
    return m_vel;
  }
  
  double PointMass::mass() const
  {
    return m_mass;
  }
  
  void PointMass::applyForce( const Vec2& force )
  {
    m_force += force;
  }
  
  void PointMass::update( double dt )
  {
    m_vel += ( m_force / m_mass - m_vel * m_friction ) * dt;
    if ( m_vel.length() > m_maxSpeed )
    {
      m_vel = m_vel.norm() * m_maxSpeed;
    }
    m_pos += m_vel * dt;
    m_force = Vec2();
  }

}
