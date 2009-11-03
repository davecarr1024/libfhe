#include <gge/App.h>
#include <gge/PyAspect.h>
using namespace gge;

class TestAspect : public Aspect
{
    public:
        TestAspect() :
            on_detach_called(false)
        {
            addFunc("on_attach",&TestAspect::on_attach,this);
            addFunc("on_detach",&TestAspect::on_detach,this);
            addFunc("set_setTest",&TestAspect::set_setTest,this);
            addFunc("get_getTest",&TestAspect::get_getTest,this);
            addFunc("msg_msgTest",&TestAspect::msg_msgTest,this);
            addFunc("load_loadTest",&TestAspect::load_loadTest,this);
        }
        
        void on_attach()
        {
            getEntity()->setVar<bool>("on_attach_called",true);
        }
        
        bool on_detach_called;
        
        void on_detach()
        {
            on_detach_called = true;
        }
        
        void set_setTest( Var val )
        {
            getEntity()->setVar<int>("setTestVal",val.get<int>());
        }
        
        Var get_getTest()
        {
            Var val;
            val.set<int>(35);
            return val;
        }
        
        void msg_msgTest( VarMap args )
        {
            getEntity()->setVar("msgTest",args.getVar<int>("val",0));
        }
        
        void load_loadTest( TiXmlHandle h )
        {
            getEntity()->setVar<bool>("loadTest_called",true);
        }
};

GGE_ASPECT(TestAspect);

int main()
{
    Var var;
    assert(var.empty());
    
    App app;
    
    EntityPtr entity = app.buildEntity("ent");
    assert(entity);
    
    AspectPtr testAspect = entity->buildAspect("Test/TestAspect");
    assert(testAspect);
    AutoPtr<TestAspect> test = testAspect.cast<TestAspect>();
    assert(test);
    
    assert(entity->getVar<bool>("on_attach_called",false) == true);
    
    test->detachFromEntity();
    assert(test->on_detach_called);
    
    test->attachToEntity(entity);
    
    entity->setVar<int>("setTest",23);
    assert(entity->getVar<int>("setTestVal",0) == 23);
    
    assert(entity->getVar<int>("getTest",0) == 35);
    
    VarMap msgTestArgs;
    msgTestArgs.setVar("val",45);
    app.publish("msgTest",msgTestArgs);
    assert(entity->getVar<int>("msgTest",0) == 45);
    
    app.load("Test/test.app");
    
    EntityPtr fileEntity = app.getEntity("fileEntity");
    assert(fileEntity);

    assert(fileEntity->getVar<int>("i",0) == 11);
    
    AutoPtr<TestAspect> fileTest = fileEntity->getAspect("Test/TestAspect").cast<TestAspect>();
    assert(fileTest);
    
    assert(fileEntity->getVar<bool>("loadTest_called",false) == true);
    
    EntityPtr scriptEnt = app.buildEntity("scriptEnt");
    scriptEnt->buildAspect("gge/PyAspect").cast<PyAspect>()->run("test.py");
    assert(scriptEnt->call<int>("crossTest") == -1);
    
    return 0;
}
