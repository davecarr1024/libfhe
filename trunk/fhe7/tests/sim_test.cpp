#include <sim/SpatialNode.h>
#include <gtest/gtest.h>
using namespace fhe;

template <size_t dim>
void spatial_node_test( const Vec<dim>& p, const Rot<dim>& r )
{
    NodePtr node( new SpatialNode<dim> );
    ASSERT_TRUE( node );
    ASSERT_EQ( Vec<dim>::ZERO, node->getVar( &SpatialNode<dim>::position ) );
    ASSERT_EQ( Rot<dim>::IDENTITY, node->getVar( &SpatialNode<dim>::rotation ) );
    ASSERT_EQ( Mat<dim>::IDENTITY, node->call( &SpatialNode<dim>::localTransform ) );
    ASSERT_EQ( Mat<dim>::IDENTITY, node->call( &SpatialNode<dim>::globalTransform ) );
    
    node->setVar( &SpatialNode<dim>::position, p );
    node->setVar( &SpatialNode<dim>::rotation, r );
    ASSERT_EQ( p, node->getVar( &SpatialNode<dim>::position ) );
    ASSERT_EQ( r, node->getVar( &SpatialNode<dim>::rotation ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ), node->call( &SpatialNode<dim>::localTransform ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ), node->call( &SpatialNode<dim>::globalTransform ) );
    
    Vec<dim> cp = p * 5.1;
    Rot<dim> cr = r * -3.14;
    
    NodePtr child( new SpatialNode<dim> );
    ASSERT_TRUE( child );
    node->attachChild( child );
    child->setVar( &SpatialNode<dim>::position, cp );
    child->setVar( &SpatialNode<dim>::rotation, cr );
    ASSERT_EQ( cp, child->getVar( &SpatialNode<dim>::position ) );
    ASSERT_EQ( cr, child->getVar( &SpatialNode<dim>::rotation ) );
    ASSERT_EQ( Mat<dim>::translation( cp ) * Mat<dim>::rotation( cr ), child->call( &SpatialNode<dim>::localTransform ) );
    ASSERT_EQ( Mat<dim>::translation( p ) * Mat<dim>::rotation( r ) * Mat<dim>::translation( cp ) * Mat<dim>::rotation( cr ), 
               child->call( &SpatialNode<dim>::globalTransform ) );
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
