#include <fhe/NodeFactory.h>
#include <gtest/gtest.h>
using namespace fhe;

class TestNode : public Node
{
    public:
        int i;
        
        void set( int _i )
        {
            i = _i;
        }
        
        virtual int get()
        {
            return i;
        }
};

FHE_NODE( TestNode )
FHE_FUNC( TestNode, set )
FHE_FUNC( TestNode, get )
FHE_VAR( TestNode, i )

TEST( node_test, factory )
{
    ASSERT_TRUE( NodeFactory::instance().build( "TestNode" ) );
}

TEST( node_test, funcs )
{
    NodePtr node( new TestNode );
    std::vector< Val > args( 1 );
    Val v;
    int i;
    
    args[0] = 1;
    ASSERT_TRUE( node->tryCall( "set", args, v ) );
    ASSERT_TRUE( node->tryCall( "get", std::vector< Val >(), v ) );
    ASSERT_TRUE( v.tryGet( i ) );
    ASSERT_EQ( 1, i );
    
    args[0] = 2;
    node->call( "set", args );
    ASSERT_EQ( 2, (int)node->call( "get", std::vector< Val >() ) );
    
    ASSERT_TRUE( node->tryCall( &TestNode::set, 3 ) );
    ASSERT_TRUE( node->tryCall( &TestNode::get, i ) );
    ASSERT_EQ( 3, i );
    
    node->call( &TestNode::set, 4 );
    ASSERT_EQ( 4, node->call( &TestNode::get ) );
    
    NodePtr child( new TestNode );
    node->attachChild( child );
    
    args[0] = 5;
    node->publish( "set", args );
    ASSERT_EQ( 5, (int)child->call( "get", std::vector< Val >() ) );
    
    node->publish( &TestNode::set, 6 );
    ASSERT_EQ( 6, child->call( &TestNode::get ) );
}

TEST( node_test, vars )
{
    NodePtr node( new TestNode );
    Val v;
    int i;
    
    ASSERT_TRUE( node->hasVar( "i" ) );
    
    ASSERT_TRUE( node->trySetVar( "i", 1 ) );
    ASSERT_TRUE( node->tryGetVar( "i", v ) );
    ASSERT_TRUE( v.tryGet( i ) );
    ASSERT_EQ( 1, i );
    
    node->setVar( "i", 2 );
    ASSERT_EQ( 2, (int)node->getVar( "i" ) );
    
    ASSERT_TRUE( node->hasVar( &TestNode::i ) );
    
    ASSERT_TRUE( node->trySetVar( &TestNode::i, 3 ) );
    ASSERT_TRUE( node->tryGetVar( &TestNode::i, i ) );
    ASSERT_EQ( 3, i );
    
    node->setVar( &TestNode::i, 4 );
    ASSERT_EQ( 4, node->getVar( &TestNode::i ) );
    
    NodePtr child( new Node );
    child->attachToParent( node );
    
    node->setVar( "i", 5 );
    ASSERT_TRUE( child->getAncestorVar( "i", v ) );
    ASSERT_EQ( 5, (int)v );
    
    node->setVar( &TestNode::i, 6 );
    ASSERT_TRUE( child->getAncestorVar( &TestNode::i, i ) );
    ASSERT_EQ( 6, i );
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

class IFoo
{
    public:
        int foo()
        {
            return 12;
        }
};

FHE_INODE( IFoo );
FHE_FUNC( IFoo, foo );

class InterfaceNode : public Node, public IFoo
{
};

FHE_NODE( InterfaceNode );
FHE_DEP( InterfaceNode, IFoo );

TEST( node_test, interface_direct )
{
    NodePtr node( new InterfaceNode );
    
    ASSERT_EQ( 12, node->call( &IFoo::foo ) );
}

TEST( node_test, interface_name )
{
    NodePtr node( new InterfaceNode );
    
    ASSERT_EQ( 12, (int)node->call( "foo", std::vector< Val >() ) );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
