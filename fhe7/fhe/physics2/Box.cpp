#include <fhe/physics2/Box.h>
#include <fhe/physics2/World.h>

namespace fhe
{
    namespace physics2
    {
        
        FHE_NODE( Box );
        FHE_DEP( Box, physics2, Shape );
        FHE_FUNC( Box, setSize );
        FHE_FUNC( Box, getSize );
        FHE_FUNC( Box, setPosition );
        FHE_FUNC( Box, getPosition );
        FHE_FUNC( Box, setRotation );
        FHE_FUNC( Box, getRotation );
        
        Box::Box() :
            m_size( 1, 1 )
        {
        }
        
        Box::~Box()
        {
        }
        
        void Box::setSize( Vec2d size )
        {
            FHE_ASSERT_MSG( !built(), "size must be set before attaching" );
            m_size = size;
        }
        
        Vec2d Box::getSize()
        {
            return m_size;
        }
        
        void Box::setPosition( Vec2d pos )
        {
            FHE_ASSERT_MSG( !built(), "position must be set before attaching" );
            m_pos = pos;
        }
        
        Vec2d Box::getPosition()
        {
            return m_pos;
        }
        
        void Box::setRotation( Rot2 rot )
        {
            FHE_ASSERT_MSG( !built(), "rotation must be set before attaching" );
            m_rot = rot;
        }
        
        Rot2 Box::getRotation()
        {
            return m_rot;
        }
        
        b2Shape* Box::build()
        {
            b2PolygonShape* shape = new b2PolygonShape;
            shape->SetAsBox( m_size.x/2, m_size.y/2, World::convert( m_pos ), World::convert( m_rot ) );
            return shape;
        }
        
    }
}
