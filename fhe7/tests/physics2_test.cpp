#include <fhe/sim/Sim.h>
#include <fhe/physics2/World.h>
#include <fhe/physics2/Body.h>
#include <fhe/physics2/Box.h>
#include <gtest/gtest.h>
using namespace fhe;

class TestNode : public sim::SpatialNode2, public sim::IUpdate
{
    public:
        void update( double time, double dtime )
        {
            if ( time > 1 )
            {
                ASSERT_LT( -1, getPosition().y );
                FHE_ASSERT( ancestorCall( &sim::Sim::shutdown ) );
            }
        }
};

FHE_NODE( TestNode );
FHE_DEP( TestNode, sim, IUpdate );
FHE_DEP( TestNode, sim, SpatialNode2 );
FHE_FUNC( TestNode, update );

TEST( physics2_test, falling_box )
{
    NodePtr sim( new sim::Sim );
    
    NodePtr world( new physics2::World );
    sim->attachChild( world );
    
    NodePtr body( new physics2::Body );
    world->attachChild( body );
    
    NodePtr shape( new physics2::Box );
    body->attachChild( shape );
    
    NodePtr test( new TestNode );
    shape->attachChild( test );
    
    sim->call( &sim::Sim::run );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
