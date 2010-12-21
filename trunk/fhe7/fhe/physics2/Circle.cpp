#include <fhe/physics2/Circle.h>
#include <fhe/physics2/World.h>

namespace fhe
{
    namespace physics2
    {
        FHE_NODE( Circle );
        FHE_DEP( Circle, physics2, Shape );
        FHE_FUNC( Circle, setRadius );
        FHE_FUNC( Circle, getRadius );
        FHE_FUNC( Circle, setPosition );
        
        Circle::Circle() :
            m_radius( 1 )
        {
        }
        
        Circle::~Circle()
        {
        }
        
        void Circle::setRadius( double radius )
        {
            FHE_ASSERT_MSG( !built(), "radius must be set before attaching" );
            m_radius = radius;
        }
        
        double Circle::getRadius()
        {
            return m_radius;
        }
        
        void Circle::setPosition( Vec2d pos )
        {
            FHE_ASSERT_MSG( !built(), "position must be set before attaching" );
            Shape::setPosition( pos );
        }
        
        b2Shape* Circle::build()
        {
            b2CircleShape* shape = new b2CircleShape;
            shape->m_radius = m_radius;
            shape->m_p = World::convert( getPosition() );
            return shape;
        }
    }
}
