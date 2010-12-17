#include <fhe/physics2/Shape.h>
#include <fhe/physics2/Body.h>
#include <fhe/physics2/World.h>

namespace fhe
{
    namespace physics2
    {
        
        FHE_INODE( Shape );
        FHE_DEP( Shape, sim, SpatialNode2 );
        FHE_DEP( Shape, core, IOnAttach );
        FHE_FUNC( Shape, onAttach );
        FHE_FUNC( Shape, onDetach );
        FHE_FUNC( Shape, setDensity );
        FHE_FUNC( Shape, getDensity );
        
        Shape::Shape() :
            m_body( 0 ),
            m_fixture( 0 ),
            m_density( 1 )
        {
        }
        
        Shape::~Shape()
        {
        }
        
        bool Shape::built() const
        {
            return m_fixture;
        }
        
        void Shape::onAttach()
        {
            FHE_ASSERT_MSG( ancestorCall( &Body::getBody, m_body ) && m_body,
                            "unable to get body" );
                            
            b2Shape* shape = build();
            FHE_ASSERT( shape );
            
            m_fixture = m_body->CreateFixture( shape, m_density );
            FHE_ASSERT( m_fixture );
            
            delete shape;
        }
        
        void Shape::onDetach()
        {
            m_body->DestroyFixture( m_fixture );
            m_fixture = 0;
            m_body = 0;
        }
        
        void Shape::setDensity( double density )
        {
            FHE_ASSERT_MSG( !built(), "density must be set before attaching" );
            m_density = density;
        }
        
        double Shape::getDensity()
        {
            return m_density;
        }
    }
}
