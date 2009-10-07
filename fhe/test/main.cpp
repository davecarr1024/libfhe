#include <cassert>
#include <cstdio>
#include <cmath>
#include <boost/bind.hpp>

#include "fhe/Node.h"
#include "fhe/FileSystem.h"
using namespace fhe;

class TestNode : public Node
{
    public:
        TestNode( const std::string& name, const std::string& type ) :
            Node( name, type )
        {
            addFunc("msg_test",&TestNode::test,this);
            addFunc("square", &TestNode::square,this);
            addFunc("noarg",&TestNode::noarg,this);
        }
        
        int msgArg;
        
        void test( int arg )
        {
            msgArg = arg;
        }
        
        float square( float x )
        {
            return x * x;
        }
        
        int noarg()
        {
            return 4;
        }
};

NODE_DECL(TestNode);
NODE_IMPL(TestNode);

void funcTest()
{
    TestNodePtr node = boost::dynamic_pointer_cast<TestNode,Node>(NodeFactory::instance().buildNode("TestNode","test"));
    assert(node);
    
    float x = node->callFunc<float,float>("square",5);
    assert(x == 25);
    
    int i = node->callFunc<int>("noarg");
    assert(i == 4);
}

void msgTest()
{
    TestNodePtr node = boost::dynamic_pointer_cast<TestNode,Node>(NodeFactory::instance().buildNode("TestNode","test"));
    assert(node);
    
    node->msgArg = 0;
    node->publish<int>("test",1);
    assert(node->msgArg == 1);
}

void nodeFactoryTest()
{
    NodePtr node = NodeFactory::instance().buildNode("TestNode","test");
    assert(node);
    assert(node->getName() == "test");
    
    TestNodePtr test = boost::dynamic_pointer_cast<TestNode,Node>(node);
    assert(test);
    
    assert(NodeFactory::instance().buildNode("Node","node"));
}

void nodeTreeTest()
{
    NodePtr root(new Node("root","Node"));
    
    root->addChild( new Node("child","Node") );
    
    assert(root->hasChild("child"));
    
    NodePtr child = root->getChild("child");
    assert(child);
    assert(child->getParent() == root);
    assert(child->getRoot() == root);
    
    NodePtr gchild = new Node("gchild","Node");
    child->addChild(gchild);
    
    assert(gchild->getNode("/") == root);
    assert(gchild->getNode(".") == gchild);
    assert(gchild->getNode("..") == child);
    assert(gchild->getNode("../..") == root);
    assert(child->getNode("gchild") == gchild);
    assert(child->getNode("../child") == child);
    assert(child->getNode("/child/gchild") == gchild);
    assert(child->getNode("./.././child/gchild/../gchild/.") == gchild);
}

void varTest()
{
    NodePtr node(new Node("node","Node"));
    
    assert(!node->hasVar<int>("i"));
    assert(!node->hasVar<float>("i"));
    node->setVar<int>("i",1);
    assert(node->hasVar<int>("i"));
    assert(!node->hasVar<float>("i"));
    assert(node->getVar<int>("i") == 1);
    
    Var v(1);
    assert(v.is<int>());
    assert(v.get<int>() == 1);
}

/*void loadTest()
{
    NodePtr node( Node::load( "test/test.xml" ) );
    
    assert(node);
    TestNodePtr testNode = boost::dynamic_pointer_cast<TestNode,Node>(node);
    assert(testNode);
    assert(node->getVar<bool>("b") == true);
    assert(node->getVar<int>("i") == 4 );
    assert(fabsf(node->getVar<float>("f") - 3.14) < 1e-4);
    assert(node->getVar<std::string>("s") == "hello");
    
    NodePtr child = node->getChild("FileChild");
    assert(child);
    assert(child->getVar<std::string>("s") == "world");
    
//     node->save("dump");
}
*/
void pythonTest()
{
    NodePtr node( new TestNode("pythonTest","TestNode") );
    
//     node->addChild( new Node("pychild","Node") );
    
    node->runScript("test/test.py");
    
/*    node->callFunc<void,std::string>("hello","world");
    std::string helloMsg = node->getVar<std::string>("hello_msg");
    assert(helloMsg == "world");
    
    node->callFunc<void>("echo");
    assert(node->getVar<bool>("didEcho"));
    
    int square = node->callFunc<int,int>("pySquare",2);
    assert( square == 4 );
    
    std::string msg = node->callFunc<std::string>("getMsg");
    assert( msg == "msg" );
    
    node->publish<std::string>("pyTest","python");
    assert(node->getVar<std::string>("pyTestVal") == "python");*/
}

int main()
{
/*    nodeTreeTest();
    nodeFactoryTest();
    varTest();
    msgTest();
    funcTest();*/
//     loadTest();
    pythonTest();
    return 0;
}
