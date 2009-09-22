#include "SGE/Core/Application.h"
#include "SGE/Core/Node.h"
#include "SGE/Core/Notification.h"
#include "SGE/Core/BoxBoundedNode.h"
#include "SGE/Core/FileServer.h"

#include "SGE/Math/SGEMath.h"
#include "SGE/Math/Vec3.h"
#include "SGE/Math/Quat.h"
#include "SGE/Math/Mat4.h"
#include "SGE/Math/AlignedBox.h"

#include "SGE/Core/BoolVar.h"
#include "SGE/Core/IntVar.h"
#include "SGE/Core/FloatVar.h"
#include "SGE/Core/StringVar.h"
#include "SGE/Core/ListVar.h"
#include "SGE/Core/DictVar.h"

#include <cstdio>
#include <cassert>
#include <iostream>

using namespace SGE;

////////////////////////////////////////////////// tool tests

void FileServerTest()
{
    assert(FileServer::instance().getFile("Test.cpp") == "Test/Test.cpp");
}

////////////////////////////////////////////////// math tests

void BoxBoundedNodeTest()
{
    NodePtr root = Application::instance().getRoot();
    
    BoxBoundedNodePtr n1 = NodeFactory::instance().buildNode("BoxBoundedNode","BoxBoundedNode").cast<BoxBoundedNode>();
    assert(n1);

    SpatialNode3Ptr n2 = NodeFactory::instance().buildNode("SpatialNode3","SpatialNode3").cast<SpatialNode3>();
    assert(n2);
    
    BoxBoundedNodePtr n3 = NodeFactory::instance().buildNode("BoxBoundedNode","BoxBoundedNode").cast<BoxBoundedNode>();
    assert(n3);
    
    root->addChild(n1);
    root->addChild(n2);
    n2->addChild(n3);
    
    assert(n1->getParent() == root);
    assert(n2->getParent() == root);
    assert(n3->getParent() == n2.cast<Node>());
    
    n1->setBox(AlignedBox(Vec3(-1,-1,-1),Vec3(1,1,1)));
    n3->setBox(AlignedBox(Vec3(-1,-1,-1),Vec3(1,1,1)));
    
    assert(n1->overlaps(n3));
    
    n1->setPosition(Vec3(10,10,0));
    n2->setPosition(Vec3(10,0,0));
    n3->setPosition(Vec3(10,0,0));
    
    assert(!n1->overlaps(n3));
    
    n2->setRotation(Quat(Vec3::UNIT_Z,Math::PI/2));
    
    assert(n1->overlaps(n3));
}

void AlignedBoxTest()
{
    AlignedBox ab(Vec3(-1,-1,-1),Vec3(1,1,1));
    assert(ab.contains(Vec3::ZERO));
    
    assert(Vec3(2,2,2).equals(ab.getSize()));
    assert(Vec3::ZERO.equals(ab.getCenter()));
    
    Vec3List vl;
    ab.getCorners(vl);
    for (Vec3List::iterator i = vl.begin(); i != vl.end(); ++i)
        assert(ab.contains(*i));
    assert(Vec3(-1,-1,-1).equals(vl[0]));
    assert(Vec3(1,1,1).equals(vl[7]));
    
    ab.expand(Vec3(3,3,3));
    assert(ab.contains(Vec3(2,2,2)));
    
    ab.expand(AlignedBox(Vec3(0,0,0),Vec3(1,1,1)),Mat4::translation(Vec3(4,4,4)));
    assert(ab.contains(Vec3(4,4,4)));

    ab.expand(AlignedBox(Vec3(-0.5,-0.5,-0.5),Vec3(0.5,0.5,0.5)),
               Mat4::translation(Vec3(5,5,5)) * Mat4::rotation(Quat(Vec3(1,1,1),Math::PI/4)));
               
    assert(ab.contains(Vec3(5.6,5.6,5.6)));
    
    AlignedBox ab1 = AlignedBox(Vec3(-1,-1,-1),Vec3(1,1,1));
    AlignedBox ab2 = AlignedBox(Vec3(-1,-1,-1),Vec3(1,1,1));
    assert(ab1.overlaps(ab2,Mat4::IDENTITY));
    assert(!ab1.overlaps(ab2,Mat4::translation(Vec3(0,3,0))));
    assert(ab1.overlaps(ab2,Mat4::translation(Vec3(0,3,0)) * Mat4::rotation(Quat(Vec3::UNIT_Z,Math::PI/4))));
    
    ab1 = AlignedBox(Vec3(-10,-1,-1),Vec3(10,1,1));
    ab2 = AlignedBox(Vec3(-1,-10,-1),Vec3(1,10,1));
    assert(ab1.overlaps(ab2,Mat4::IDENTITY));
    assert(!ab1.overlaps(ab2,Mat4::translation(Vec3(0,0,5))));
    assert(ab1.overlaps(ab2,Mat4::translation(Vec3(0,0,5)) * Mat4::rotation(Quat(Vec3::UNIT_X,Math::PI/2))));
}

void Mat4Test()
{
    Vec3 v(5,6,7);
    
    Vec3 t(10,5,-2);
    Quat r(Vec3(0,2,3),2);
    Vec3 s(4,-2,3);
    
    assert((v + t).equals(Mat4::translation(t) * v));

    assert((r * v).equals(Mat4::rotation(r) * v));
    
    assert(Vec3(v.x * s.x, v.y * s.y, v.z * s.z).equals(Mat4::scale(s) * v));
    
    Vec3 simple = Mat4::scale(s) * (Mat4::rotation(r) * (Mat4::translation(t) * v));
    Vec3 complex = (Mat4::scale(s) * Mat4::rotation(r) * Mat4::translation(t)) * v;
    assert(simple.equals(complex));
    
    Mat4 bigMat = Mat4::scale(s) * Mat4::rotation(r) * Mat4::translation(t);
    assert(v.equals((bigMat * bigMat.inverse()) * v));
}

void QuatTest()
{
    Quat q(Vec3::UNIT_Z,Math::PI/2);
    
    assert(Vec3(0,1,0).equals(q * Vec3::UNIT_X));
    
    assert(Vec3(0,-1,0).equals(q * q * q * Vec3::UNIT_X));

    assert(Vec3(0,-1,0).equals(q.inverse() * Vec3::UNIT_X));
}

void Vec3Test()
{
    Vec3 v1(1,2,3);
    Vec3 v2(4,6,8);
    
    assert(Vec3(5,8,11).equals(v1 + v2));
    assert(Math::equal(1,v1[0]));
    v1[2] = 4;
    assert(Math::equal(4,v1[2]));
    
    assert(Vec3(-3,-4,-4).equals(v1 - v2));
    
    assert(Vec3(5,10,20).equals(v1 * 5));
    
    assert(Vec3(1,1.5,2).equals(v2 / 4.0));
    
    assert(Vec3(-4,-6,-8).equals(-v2));

    v1 += Vec3(10,5,-10);
    assert(Vec3(11,7,-6).equals(v1));
    
    v1 -= Vec3(5,5,6);
    assert(Vec3(6,2,-12).equals(v1));
    
    v1 *= 2;
    assert(Vec3(12,4,-24).equals(v1));
    
    v1 /= 4;
    assert(Vec3(3,1,-6).equals(v1));
    
    v1 = Vec3(3,4,0);
    assert(Math::equal(5,v1.length()));
    
    assert(Math::equal(1,v1.norm().length()));
    
    v1 = Vec3(14,5,-6);
    v2 = Vec3(3,-2,1);
    assert(Math::equal(26,v1.dot(v2)));
    
    v1 = Vec3(100,0,0);
    v2 = Vec3(50,50,0);
    assert(Vec3(50,0,0).equals(v1.project(v2)));
    
    v1 = Vec3(1,0,0);
    v2 = Vec3(0,0,1);
    Quat q = v1.getRotTo(v2);
    Vec3 axis;
    float angle;
    q.toAxisAngle(axis,angle);
    if (axis.dot(Vec3::UNIT_Y) < 0)
    {
        axis *= -1;
        angle *= -1;
    }
    assert(Math::equal(angle,Math::PI/-2));
    assert(Vec3::UNIT_Y.equals(axis));
    
    v1 = Vec3(10,20,-30);
    v2 = v1.makePerp();
    assert(Math::equal(0,v1.dot(v2)));
    
    v2 = Vec3(0,40,30);
    assert(Vec3(5,30,0).equals(v1.lerp(v2,0.5)));
}

////////////////////////////////////////////////// node tests

void VarTest()
{
    NodePtr root = Application::instance().getRoot();
    assert( root );
    
    root->setVar( "bool", new BoolVar( true ) );
    VarPtr testBoolVar = root->getVar( "bool" );
    assert( testBoolVar );
    assert( testBoolVar->getType() == Var::BOOL );
    BoolVarPtr boolVar = testBoolVar.cast<BoolVar>();
    assert( boolVar );
    assert( boolVar->get() );
    
    root->setVar( "int", new IntVar( 42 ) );
    IntVarPtr intVar = root->getVar( "int" ).cast<IntVar>();
    assert( intVar );
    assert( intVar->get() == 42 );
    
    root->setVar( "float", new FloatVar( 3.14 ) );
    FloatVarPtr floatVar = root->getVar( "float" ).cast<FloatVar>();
    assert( floatVar );
    assert( Math::equal( floatVar->get(), 3.14 ) );
    
    root->setVar( "string", new StringVar( "test" ) );
    StringVarPtr stringVar = root->getVar( "string" ).cast<StringVar>();
    assert( stringVar );
    assert( stringVar->get() == "test" );
    
    ListVarPtr list( new ListVar );
    list->append( new BoolVar( true ) );
    list->append( new IntVar( 12 ) );
    list->append( new FloatVar( 2.5 ) );
    list->append( new StringVar( "in a list" ) );
    root->setVar( "list", list );
    
    ListVarPtr testList = root->getVar( "list" ).cast<ListVar>();
    assert( testList );
    assert( testList->size() == 4 );
    
    boolVar = testList->get(0).cast<BoolVar>();
    assert( boolVar );
    assert( boolVar->get() );
    
    intVar = testList->get(1).cast<IntVar>();
    assert( intVar );
    assert( intVar->get() == 12 );
    
    floatVar = testList->get(2).cast<FloatVar>();
    assert( floatVar );
    assert( Math::equal( floatVar->get(), 2.5 ) );
    
    stringVar = testList->get(3).cast<StringVar>();
    assert( stringVar );
    assert( stringVar->get() == "in a list" );
    
    DictVarPtr dict( new DictVar );
    dict->set( "b", new BoolVar( false ) );
    dict->set( "i", new IntVar( -1 ) );
    dict->set( "f", new FloatVar( -0.2 ) );
    dict->set( "s", new StringVar( "val" ) );
    root->setVar( "dict", dict );
    
    DictVarPtr testDict = root->getVar( "dict" ).cast<DictVar>();
    assert( testDict );
    assert( testDict->size() == 4 );

    boolVar = testDict->get( "b" ).cast<BoolVar>();
    assert( boolVar );
    assert( !boolVar->get() );
    
    intVar = testDict->get( "i" ).cast<IntVar>();
    assert( intVar );
    assert( intVar->get() == -1 );
    
    floatVar = testDict->get( "f" ).cast<FloatVar>();
    assert( floatVar );
    assert( Math::equal( floatVar->get(), -0.2 ) );
    
    stringVar = testDict->get( "s" ).cast<StringVar>();
    assert( stringVar );
    assert( stringVar->get() == "val" );
}

void DynamicNodeTest()
{
    assert(NodeFactory::instance().buildNode("DynamicNode","DynamicNode"));
}

class TestNotification : public Notification
{
    private:
        std::string m_msg;
        
    public:
        TestNotification(const std::string& msg) :
            m_msg(msg)
        {
        }
        
        std::string getMsg() const
        {
            return m_msg;
        }
};

class TestNode : public Node
{
    private:
        std::string m_msg;
    
    public:
        TestNode() : 
            Node() 
        {
            SUBSCRIBE_MSG(TestNode, TestNotification);
        }
        
//         ON_MSG(TestNotification, msg)
//         {
//             m_msg = msg.getMsg();
//         }

        ON_MSG_DECL(TestNotification)
        
        int foo() 
        { 
            return 4; 
        }
        
        std::string getMsg() const
        {
            return m_msg;
        }
};

ON_MSG_IMPL(TestNode, TestNotification, msg)
{
    m_msg = msg.getMsg();
}

REGISTER_NODE_TYPE(TestNode);
DECLARE_NODE_PTR(TestNode);

void NodeTest() 
{
    NodePtr root = Application::instance().getRoot();
    assert(root);
    
    NodePtr child = NodeFactory::instance().buildNode("TestNode","child");
    assert(child);
    assert(child->getName() == "child");
    
    root->addChild(child);
    
    NodePtr test = root->getChild(child->getName());
    assert(test);
    assert(test->getName() == child->getName());

    assert(child->getPath() == std::string("/") + child->getName());
    assert(child->getObject(child->getPath()) == child);

    TestNodePtr testNode = test.cast<TestNode>();
    assert(testNode);
    assert(testNode->foo() == 4);
    
    assert(test->getObject("/") == root);
    assert(test->getObject(".") == test);
    assert(test->getObject("..") == root);
    assert(root->getObject(test->getName()) == test);
    assert(test->getObject(std::string("../") + test->getName()) == test);

    root->publish(TestNotification("hello"));
    assert(testNode->getMsg() == "hello");
    
    test->unsubscribe<TestNotification>();
    root->publish(TestNotification("world"));
    assert(testNode->getMsg() == "hello");
    
    assert(test->getFirstAncestorOfType<Node>() == root);
    
    TestNodeList nodes;
    root->getDescendantsOfType<TestNode>(nodes);
    assert(nodes[0] == testNode);
    
    root->removeChild(test->getName());
    assert(!root->hasChild(test->getName()));
}

void LoadTest()
{
    NodePtr root = Application::instance().getRoot();
    assert( root );
    
    root->load( "TestApp.xml" );
    NodePtr node = root->getChild( "LoadTest" );
    assert( node );
    
    TestNodePtr testNode = node.cast<TestNode>();
    assert( testNode );
    
    BoolVarPtr boolVar = testNode->getVar( "b" ).cast<BoolVar>();
    assert( boolVar );
    assert( boolVar->get() );
    
    ListVarPtr listVar = testNode->getVar( "list" ).cast<ListVar>();
    assert( listVar );
    assert( listVar->size() == 2 );
    
    IntVarPtr intVar = listVar->get( 1 ).cast<IntVar>();
    assert( intVar );
    assert( intVar->get() == 12 );
    
    DictVarPtr dictVar = testNode->getVar( "d" ).cast<DictVar>();
    assert( dictVar );
    assert( dictVar->size() == 2 );
    
    FloatVarPtr floatVar = dictVar->get( "f" ).cast<FloatVar>();
    assert( floatVar );
    assert( Math::equal( floatVar->get(), 3.14 ) );
    
    StringVarPtr stringVar = dictVar->get( "s" ).cast<StringVar>();
    assert( stringVar );
    assert( stringVar->get() == "hello" );
}

int main() 
{
    NodeTest();
    DynamicNodeTest();
    VarTest();
    LoadTest();
    Vec3Test();
    QuatTest();
    Mat4Test();
    AlignedBoxTest();
    BoxBoundedNodeTest();
    FileServerTest();
}
