#include <fhe/NodeFactory.h>
#include <gtest/gtest.h>
using namespace fhe;

TEST( node_test, tree )
{
    NodePtr root( new Node ), child( new Node );
    root->attachChild( child );
    ASSERT_TRUE( root->hasChild( child ) );
    ASSERT_EQ( child, *root->childrenBegin() );
    ASSERT_EQ( root, child->parent() );
    ASSERT_EQ( root, child->root() );
    child->detachFromParent();
    ASSERT_FALSE( child->parent() );
    ASSERT_FALSE( root->hasChild( child ) );
}

class IVar
{
    public:
        int var;
};

class IFuncs
{
    private:
        int m_i;
        
    public:
        void set( int i )
        {
            m_i = i;
        }
        
        int get()
        {
            return m_i;
        }
};

class TestNode : public Node, public IVar, public IFuncs
{
};

FHE_NODE( TestNode );
FHE_FUNC( TestNode, set );
FHE_FUNC( TestNode, get );

TEST( node_test, vars )
{
    NodePtr test( new TestNode );
    test->setVar( &IVar::var, 1 );
    ASSERT_EQ( 1, test->getVar( &IVar::var ) );
}

TEST( node_test, funcs )
{
    NodePtr test( new TestNode );
    test->call( &IFuncs::set, 2 );
    ASSERT_EQ( 2, test->call( &IFuncs::get ) );
}

TEST( node_test, publish )
{
    NodePtr root( new Node ), child( new TestNode );
    child->attachToParent( root );
    root->publish( &IFuncs::set, 3 );
    ASSERT_EQ( 3, child->call( &IFuncs::get ) );
}

TEST( node_test, name_funcs_fac )
{
    NodePtr test = NodeFactory::instance().build( "TestNode" );
    ASSERT_TRUE( test );
    test->call( "set", 4 );
    ASSERT_EQ( 4, test->retCall<int>( "get" ) );
}

TEST( node_test, name_funcs_ctor )
{
    NodePtr test( new TestNode );
    ASSERT_TRUE( test );
    test->call( "set", 5 );
    ASSERT_EQ( 5, test->retCall<int>( "get" ) );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
