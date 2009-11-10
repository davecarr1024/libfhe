#include <fhe/Aspect.h>
using namespace fhe;

class TestAspect : public Aspect
{
    public:
        FHE_FUNC_DECL(testFunc);
        FHE_FUNC_DECL(inheritTest);
};

FHE_ASPECT(TestAspect,Aspect);

FHE_FUNC_IMPL(TestAspect,testFunc)
{
    return Var::build<int>(arg.get<int>()*2);
}

FHE_FUNC_IMPL(TestAspect,inheritTest)
{
    getEntity()->setVar<bool>("parent_inheritTest",true);
    return Var();
}

class ChildAspect : public TestAspect
{
    public:
        FHE_FUNC_DECL(msg_msgTest);
        FHE_FUNC_DECL(inheritTest);
};

FHE_ASPECT(ChildAspect,TestAspect);

FHE_FUNC_IMPL(ChildAspect,msg_msgTest)
{
    getEntity()->setRawVar("msgTest",arg);
    return Var();
}

FHE_FUNC_IMPL(ChildAspect,inheritTest)
{
    assert(getEntity()->getVar<bool>("parent_inheritTest"));
    getEntity()->setVar<bool>("child_inheritTest",true);
    return Var();
}

int main()
{
    EntityPtr root = new Entity("root");
    assert(root);

    AutoPtr<TestAspect> testAspect = root->buildAspect("Test/TestAspect").cast<TestAspect>();
    assert(testAspect);
    
    assert(root->call("testFunc",Var::build<int>(2)).get<int>() == 4);
    
    root->call("inheritTest");
    assert(root->getVar<bool>("parent_inheritTest"));
    
    EntityPtr child = root->buildChild("child");
    assert(child);
    
    AutoPtr<ChildAspect> childAspect = child->buildAspect("Test/ChildAspect").cast<ChildAspect>();
    assert(childAspect);
    
    child->call("inheritTest");
    assert(child->getVar<bool>("child_inheritTest"));
    assert(child->getVar<bool>("parent_inheritTest"));
    
    assert(child->hasFunc("testFunc"));
    assert(child->call("testFunc",Var::build<int>(5)).get<int>() == 10);

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
