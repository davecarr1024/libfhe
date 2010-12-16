#include <fhe/sim/SpatialNode.h>
#include <gtest/gtest.h>
using namespace fhe;
using namespace sim;

template <size_t dim>
void spatial_node_test( const Vec<dim>& p, const Rot<dim>& r )
{
    NodePtr node( new SpatialNode<dim> );
    ASSERT_TRUE( node );
    ASSERT_EQ( Vec<dim>::ZERO, node->call( &SpatialNode<dim>::getPosition ) );
    ASSERT_EQ( Rot<dim>::IDENTITY, node->call( &SpatialNode<dim>::getRotation ) );
    ASSERT_EQ( Mat<dim>::IDENTITY, node->call( &SpatialNode<dim>::getGlobalTransform ) );
    ASSERT_EQ( Mat<dim>::IDENTITY, node->call( &SpatialNode<dim>::getGlobalTransform ) );
    
    node->call( &SpatialNode<dim>::setPosition, p );
    node->call( &SpatialNode<dim>::setRotation, r );
    ASSERT_EQ( p, node->call( &SpatialNode<dim>::getPosition ) );
    ASSERT_EQ( r, node->call( &SpatialNode<dim>::getRotation ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ), node->call( &SpatialNode<dim>::getLocalTransform ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ), node->call( &SpatialNode<dim>::getGlobalTransform ) );
    
    Vec<dim> cp = p * 5.1;
    Rot<dim> cr = r * -3.14;
    
    NodePtr child( new SpatialNode<dim> );
    ASSERT_TRUE( child );
    node->attachChild( child );
    child->call( &SpatialNode<dim>::setPosition, cp );
    child->call( &SpatialNode<dim>::setRotation, cr );
    ASSERT_EQ( cp, child->call( &SpatialNode<dim>::getPosition ) );
    ASSERT_EQ( cr, child->call( &SpatialNode<dim>::getRotation ) );
    ASSERT_EQ( Mat<dim>::translation( cp ) * Mat<dim>::rotation( cr ), child->call( &SpatialNode<dim>::getLocalTransform ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ) * Mat<dim>::translation( cp ) * Mat<dim>::rotation( cr ), 
               child->call( &SpatialNode<dim>::getGlobalTransform ) );
}

TEST( sim_test, SpatialNode2 )
{
    spatial_node_test( Vec2( 3, 4 ), Rot2::fromDegrees( 55 ) );
}

TEST( sim_test, SpatialNode3 )
{
    spatial_node_test( Vec3( 3, 4, 8 ), Rot3( Vec3( -1, 2, 8 ), 1.56 ) );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
