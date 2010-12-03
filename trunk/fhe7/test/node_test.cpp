#include <fhe/NodeFactory.h>
#include <gtest/gtest.h>
using namespace fhe;

class TestNode : public Node
{
    public:
        int m_i;
        
        void set( int i )
        {
            m_i = i;
        }
        
        virtual int get()
        {
            return m_i;
        }
};

FHE_NODE( TestNode )
FHE_FUNC( TestNode, set )
FHE_FUNC( TestNode, get )
FHE_VAR( TestNode, m_i )

TEST( node_test, factory )
{
    ASSERT_TRUE( NodeFactory::instance().build( "TestNode" ) );
}

TEST( node_test, name_funcs )
{
    NodePtr node( new TestNode );
    
    std::vector< Val > args;
    args.push_back( 1 );
    node->call( "set", args );

    ASSERT_EQ( 1, (int)node->call( "get", std::vector< Val >() ) );
}

TEST( node_test, direct_funcs )
{
    NodePtr node( new TestNode );
    
    node->call( &TestNode::set, 2 );
    ASSERT_EQ( 2, node->call( &TestNode::get ) );
}

TEST( node_test, name_vars )
{
    NodePtr node( new TestNode );
    
    node->set( "m_i", 3 );
    ASSERT_EQ( 3, (int)node->get( "m_i" ) );
}

TEST( node_test, direct_vars )
{
    NodePtr node( new TestNode );
    
    node->set( &TestNode::m_i, 4 );
    ASSERT_EQ( 4, node->get( &TestNode::m_i ) );
}

class ChildNode : public TestNode
{
    public:
        int get()
        {
            return TestNode::get() * 2;
        }
};

FHE_NODE( ChildNode );
FHE_DEP( ChildNode, TestNode );
FHE_FUNC( ChildNode, get );

TEST( node_test, inheritance_direct )
{
    NodePtr node( new ChildNode );
    
    node->call( &TestNode::set, 1 );
    ASSERT_EQ( 2, node->call( &TestNode::get ) );
}

TEST( node_test, inheritance_name )
{
    NodePtr node( new ChildNode );
    
    std::vector< Val > args;
    args.push_back( 2 );
    node->call( "set", args );
    ASSERT_EQ( 4, (int)node->call( "get", std::vector< Val >() ) );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
