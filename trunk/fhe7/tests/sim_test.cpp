#include <fhe/sim/SpatialNode.h>
#include <gtest/gtest.h>
using namespace fhe;
using namespace sim;

template <size_t dim, typename T>
void spatial_node_test( const Vec<dim,T>& p, const Rot<dim>& r )
{
    NodePtr node( new SpatialNode<dim,T> );
    ASSERT_TRUE( node );
    ASSERT_EQ( (Vec<dim,T>::ZERO), (node->call( &SpatialNode<dim,T>::getPosition )) );
    ASSERT_EQ( Rot<dim>::IDENTITY, node->call( &SpatialNode<dim,T>::getRotation ) );
    ASSERT_EQ( Mat<dim>::IDENTITY, node->call( &SpatialNode<dim,T>::getGlobalTransform ) );
    ASSERT_EQ( Mat<dim>::IDENTITY, node->call( &SpatialNode<dim,T>::getGlobalTransform ) );
    
    node->call( &SpatialNode<dim,T>::setPosition, p );
    node->call( &SpatialNode<dim,T>::setRotation, r );
    ASSERT_EQ( p, node->call( &SpatialNode<dim,T>::getPosition ) );
    ASSERT_EQ( r, node->call( &SpatialNode<dim,T>::getRotation ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ), node->call( &SpatialNode<dim,T>::getLocalTransform ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ), node->call( &SpatialNode<dim,T>::getGlobalTransform ) );
    
    Vec<dim,T> cp = p * 5.1;
    Rot<dim> cr = r * -3.14;
    
    NodePtr child( new SpatialNode<dim,T> );
    ASSERT_TRUE( child );
    node->attachChild( child );
    child->call( &SpatialNode<dim,T>::setPosition, cp );
    child->call( &SpatialNode<dim,T>::setRotation, cr );
    ASSERT_EQ( cp, child->call( &SpatialNode<dim,T>::getPosition ) );
    ASSERT_EQ( cr, child->call( &SpatialNode<dim,T>::getRotation ) );
    ASSERT_EQ( Mat<dim>::translation( cp ) * Mat<dim>::rotation( cr ), child->call( &SpatialNode<dim,T>::getLocalTransform ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ) * Mat<dim>::translation( cp ) * Mat<dim>::rotation( cr ), 
               child->call( &SpatialNode<dim,T>::getGlobalTransform ) );
}

TEST( sim_test, SpatialNode2 )
{
    spatial_node_test( Vec2d( 3, 4 ), Rot2::fromDegrees( 55 ) );
}

TEST( sim_test, SpatialNode3 )
{
    spatial_node_test( Vec3d( 3, 4, 8 ), Rot3( Vec3d( -1, 2, 8 ), 1.56 ) );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
