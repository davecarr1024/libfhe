#include <fhe/Aspect.h>
#include <fhe/math/Mat3.h>
#include <fhe/math/Vec2.h>
#include <fhe/math/Rot.h>
#include <fhe/VarList.h>
using namespace fhe;

class TestAspect : public Aspect
{
    public:
        FHE_FUNC_DECL(testFunc);
        FHE_FUNC_DECL(inheritTest);
        FHE_FUNC_DECL(get_getTest);
        FHE_FUNC_DECL(set_setTest);
};

FHE_ASPECT(TestAspect,Aspect);

FHE_FUNC_IMPL(TestAspect,set_setTest)
{
    getEntity()->setRawVar("setTestVal",arg);
    return Var();
}

FHE_FUNC_IMPL(TestAspect,get_getTest)
{
    return Var::build<std::string>("got");
}

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

void mathTest()
{
    Vec2 v(5,10);
    
    Vec2 t(12,24);
    Rot r(Math::radians(90));
    Vec2 s(2,4);
    
    assert((v + t).equals(Vec2(17,34)));
    assert((r * v).equals(Vec2(-10,5)));
    assert((v * s).equals(Vec2(10,40)));
    
    Mat3 mt = Mat3::translation(t);
    assert((v + t).equals(mt * v));
    
    Mat3 mr = Mat3::rotation(r);
    assert((r * v).equals(mr * v));
    
    Mat3 ms = Mat3::scale(s);
    assert((v * s).equals(ms * v));
    
    Vec2 direct = s * ( r * ( t + v ) );
    Vec2 simple = ms * ( mr * ( mt * v ) );
    assert(direct.equals(simple));
    
    Vec2 complex = ( ms * mr * mt ) * v;
    assert(direct.equals(complex));
    
    Mat3 mti = mt.inverse(), mri = mr.inverse(), msi = ms.inverse();
    
    assert((v - t).equals(mti * v));
    assert((-r * v).equals(mri * v));
    assert((v / s).equals(msi * v));
    
    Vec2 idirect = ( -r * ( v - t ) ) / s;
    Vec2 isimple = msi * ( mri * ( mti * v ) );
    assert(idirect.equals(isimple));
    
    Vec2 icomplex = (msi * mri * mti ) * v;
    assert(idirect.equals(icomplex));
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
    
    assert(root->hasVar<std::string>("getTest"));
    assert(root->getVar<std::string>("getTest","") == "got");
    
    root->setVar<std::string>("setTest","hello");
    assert(root->getVar<std::string>("setTestVal","") == "hello");
    
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
    VarList l = fileEnt->getVar<VarList>("l");
    assert(l.length() == 2);
    assert(l.getVar<int>(0) == 12);
    assert(l.getVar<std::string>(1) == "hi");
    
    assert(fileEnt->hasAspect("Test/TestAspect"));
    
    mathTest();
    
    return 0;
}
