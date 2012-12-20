#include <sabo/Pin.h>

namespace sabo
{

  Pin::Pin( Body* body, const PointMass::Ptr& pm ) :
    Base( body ),
    m_pm( pm ),
    m_basePos( m_pm->pos() )
  {
  }
  
  Pin::~Pin()
  {
  }
  
  void Pin::update( double dt )
  {
    Vec2 diff = m_basePos - m_pm->pos();
    Vec2 projVel = m_pm->vel().project( diff );
    Vec2 projForce = diff * m_propGain;
    Vec2 diffForce = projVel * m_diffGain;
    Vec2 force = projForce - diffForce;
    m_pm->applyForce( force );
  }

}
