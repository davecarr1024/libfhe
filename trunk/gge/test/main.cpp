#include <gge/App.h>
using namespace gge;

class TestAspect : public Aspect
{
    public:
        TestAspect() :
            on_detach_called(false)
        {
            addFunc("on_attach",this,&TestAspect::on_attach);
            addFunc("on_detach",this,&TestAspect::on_detach);
            addFunc("set_setTest",this,&TestAspect::set_setTest);
            addFunc("get_getTest",this,&TestAspect::get_getTest);
            addFunc("msg_msgTest",this,&TestAspect::msg_msgTest);
            addFunc("load_loadTest",this,&TestAspect::load_loadTest);
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
            getEntity()->setVar("msgTest",args.getVar<int>("val"));
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
    
    AspectPtr aspect = entity->buildAspect("Aspect");
    assert(aspect);
    
    AspectPtr testAspect = entity->buildAspect("TestAspect");
    assert(testAspect);
    AutoPtr<TestAspect> test = testAspect.cast<TestAspect>();
    assert(test);
    
    assert(entity->getVar<bool>("on_attach_called",false) == true);
    
    test->detachFromEntity();
    assert(test->on_detach_called);
    
    test->attachToEntity(entity);
    
    entity->setVar<int>("setTest",23);
    assert(entity->getVar<int>("setTestVal") == 23);
    
    assert(entity->getVar<int>("getTest") == 35);
    
    VarMap msgTestArgs;
    msgTestArgs.setVar("val",45);
    app.publish("msgTest",msgTestArgs);
    assert(entity->getVar<int>("msgTest") == 45);
    
    app.load("test/test.app");
    
    EntityPtr fileEntity = app.getEntity("fileEntity");
    assert(fileEntity);

    assert(fileEntity->getVar<int>("i") == 11);
    
    AutoPtr<TestAspect> fileTest = fileEntity->getAspect("TestAspect").cast<TestAspect>();
    assert(fileTest);
    
    assert(fileEntity->getVar<bool>("loadTest_called",false) == true);
    
    app.buildEntity("scriptEnt")->buildAspect("Aspect")->runScript("test/test.py");
    
    return 0;
}
