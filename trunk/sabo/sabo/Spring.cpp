#include <sabo/Spring.h>

namespace sabo
{

  Spring::Spring( Body* body, const PointMass::Ptr& p1, const PointMass::Ptr& p2 ) :
    Base( body ),
    m_p1( p1 ),
    m_p2( p2 ),
    m_propGain( 100 ),
    m_diffGain( 10 ),
    m_baseDist( ( m_p1->pos() - m_p2->pos() ).length() )
  {
  }
  
  Spring::~Spring()
  {
  }
  
  void Spring::update( double dt )
  {
    Vec2 diff = m_p2->pos() - m_p1->pos();
    double diffLength = diff.length();
    Vec2 normDiff = diff.norm();
    Vec2 propForce = normDiff * ( ( diffLength - m_baseDist ) * m_propGain );
    
    Vec2 projVel1 = m_p1->vel().project( normDiff );
    Vec2 projVel2 = m_p2->vel().project( normDiff );
    Vec2 velDiff = projVel2 - projVel1;
    Vec2 diffForce = velDiff * m_diffGain;
    
    Vec2 force = propForce + diffForce;
    
    m_p1->applyForce( force );
    m_p2->applyForce( force );
  }

}
