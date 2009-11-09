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

    AutoPtr<TestAspect> testAspect = root->buildAspect("Test/TestAspect").cast<TestAspect>();
    assert(testAspect);
    
    assert(root->call("testFunc",Var::build<int>(2)).get<int>() == 4);
    
    EntityPtr child = root->buildChild("child");
    assert(child);
    
    AutoPtr<ChildAspect> childAspect = child->buildAspect("Test/ChildAspect").cast<ChildAspect>();
    assert(childAspect);
    
    assert(child->hasFunc("testFunc"));
    assert(child->call("testFunc",Var::build<int>(5)).get<int>() == 10);

    child->m_children.begin();
    root->m_children.begin();
    child->getRoot()->m_children.begin();
    child->getRoot()->publish("msgTest");
    
    EntityPtr fileEnt = root->loadChild("Test/test.app");
    assert(fileEnt);
    
    assert(fileEnt->getVar<bool>("b"));
    assert(fileEnt->getVar<int>("i") == 13);
    assert(fileEnt->getVar<float>("f") == 2);
    assert(fileEnt->getVar<std::string>("s") == "hello");
    VarMap d = fileEnt->getVar<VarMap>("d");
    assert(d.getVar<int>("i") == 22);
    assert(d.getVar<std::string>("s") == "dictstring");
    
    assert(fileEnt->hasAspect("Test/TestAspect"));
    
    return 0;
}
