#include <fhe/sim/Sim.h>
#include <fhe/sim/IUpdate.h>
#include <fhe/physics/World.h>
#include <fhe/physics/Box.h>
#include <gtest/gtest.h>
using namespace fhe;

class TestNode : public Node, public sim::IUpdate
{
    public:
        void update( double time, double dtime )
        {
            if ( time > 2 )
            {
                NodePtr parent = this->parent();
                ASSERT_TRUE( parent );
                Vec3 pos;
                ASSERT_TRUE( parent->tryCall( &physics::Body::getPosition, pos ) );
                ASSERT_LT( Math::abs( pos.z ), 1 );
                ancestorCall( &sim::Sim::shutdown );
            }
        }
};

FHE_NODE( TestNode );
FHE_DEP( TestNode, sim, IUpdate );
FHE_FUNC( TestNode, update );

TEST( physics_test, falling_box )
{
    NodePtr sim( new sim::Sim );
    
    NodePtr world( new physics::World );
    sim->attachChild( world );
    
    NodePtr box( new physics::Box );
    box->call( &physics::Body::setPosition, Vec3( 0, 0, 20 ) );
    world->attachChild( box );
    
    NodePtr test( new TestNode );
    box->attachChild( test );
    
    sim->call( &sim::Sim::run );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
