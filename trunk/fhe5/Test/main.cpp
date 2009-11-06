#include <fhe/Aspect.h>
using namespace fhe;

class TestAspect : public Aspect
{
    public:
        FHE_FUNC(TestAspect,testFunc);
};

FHE_ASPECT(TestAspect,Aspect);

Var TestAspect::testFunc( const Var& arg )
{
    return Var::build<int>(arg.get<int>()*2);
}

class ChildAspect : public TestAspect
{
    public:
        FHE_FUNC(ChildAspect,msg_msgTest);
};

FHE_ASPECT(ChildAspect,TestAspect);

Var ChildAspect::msg_msgTest( const Var& arg )
{
    getEntity()->setRawVar("msgTest",arg);
    return Var();
}

int main()
{
    EntityPtr root = new Entity("root");
    assert(root);

    AutoPtr<TestAspect> testAspect = root->buildAspect("TestAspect").cast<TestAspect>();
    assert(testAspect);
    
    assert(root->call("testFunc",Var::build<int>(2)).get<int>() == 4);
    
    EntityPtr child = root->buildChild("child");
    assert(child);
    
    AutoPtr<ChildAspect> childAspect = child->buildAspect("ChildAspect").cast<ChildAspect>();
    assert(childAspect);
    
    assert(child->call("testFunc",Var::build<int>(5)).get<int>() == 10);

    child->m_children.begin();
    root->m_children.begin();
    child->getRoot()->m_children.begin();
    child->getRoot()->publish("msgTest");
    
    return 0;
}
