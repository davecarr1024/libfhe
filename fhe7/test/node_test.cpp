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
        
        int get()
        {
            return m_i;
        }
};

FHE_NODE( TestNode )
FHE_FUNC( TestNode, set )
FHE_FUNC( TestNode, get )
FHE_VAR( TestNode, m_i )

TEST( node_test, name_funcs )
{
    NodePtr node( NodeFactory::instance().build( "TestNode" ) );
    ASSERT_TRUE( node );
    
    std::vector< Val > args;
    args.push_back( 1 );
    node->call( "set", args );

    ASSERT_EQ( 1, (int)node->call( "get", std::vector< Val >() ) );
}

TEST( node_test, direct_funcs )
{
    NodePtr node( NodeFactory::instance().build( "TestNode" ) );
    ASSERT_TRUE( node );
    
    node->call( &TestNode::set, 2 );
    ASSERT_EQ( 2, node->call( &TestNode::get ) );
}

TEST( node_test, name_vars )
{
    NodePtr node( NodeFactory::instance().build( "TestNode" ) );
    ASSERT_TRUE( node );
    
    node->set( "m_i", 3 );
    ASSERT_EQ( 3, (int)node->get( "m_i" ) );
}

TEST( node_test, direct_vars )
{
    NodePtr node( NodeFactory::instance().build( "TestNode" ) );
    ASSERT_TRUE( node );
    
    node->set( &TestNode::m_i, 4 );
    ASSERT_EQ( 4, node->get( &TestNode::m_i ) );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
