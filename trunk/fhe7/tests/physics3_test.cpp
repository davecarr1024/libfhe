#include <fhe/sim/Sim.h>
#include <fhe/sim/IUpdate.h>
#include <fhe/physics3/World.h>
#include <fhe/physics3/Box.h>
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
                Vec3d pos;
                ASSERT_TRUE( parent->tryCall( &physics3::Body::getPosition, pos ) );
                ASSERT_LT( Math::abs( pos.z ), 1 );
                ancestorCall( &sim::Sim::shutdown );
            }
        }
};

FHE_NODE( TestNode );
FHE_DEP( TestNode, sim, IUpdate );
FHE_FUNC( TestNode, update );

TEST( physics3_test, falling_box )
{
    NodePtr sim( new sim::Sim );
    
    NodePtr world( new physics3::World );
    sim->attachChild( world );
    
    NodePtr box( new physics3::Box );
    box->call( &physics3::Body::setPosition, Vec3d( 0, 0, 20 ) );
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
