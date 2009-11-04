#include <gge/App.h>
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
        
        Var on_attach( const Var& arg )
        {
            getEntity()->setVar<bool>("on_attach_called",true);
            return Var();
        }
        
        bool on_detach_called;
        
        Var on_detach( const Var& arg )
        {
            on_detach_called = true;
            return Var();
        }
        
        Var set_setTest( const Var& arg )
        {
            getEntity()->setVar<int>("setTestVal",arg.get<int>(0));
            return Var();
        }
        
        Var get_getTest( const Var& arg )
        {
            Var val;
            val.set<int>(35);
            return val;
        }
        
        Var msg_msgTest( const Var& arg )
        {
            getEntity()->setVar("msgTest",arg.get<int>(0));
            return Var();
        }
        
        Var load_loadTest( const Var& arg )
        {
            getEntity()->setVar<bool>("loadTest_called",true);
            return Var();
        }
};

GGE_ASPECT(TestAspect);

int main()
{
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
    
    app.publish("msgTest",Var::build<int>(45));
    assert(entity->getVar<int>("msgTest",0) == 45);
    
    app.load("Test/test.app");
    
    EntityPtr fileEntity = app.getEntity("fileEntity");
    assert(fileEntity);

    assert(fileEntity->getVar<int>("i",0) == 11);
    
    AutoPtr<TestAspect> fileTest = fileEntity->getAspect("Test/TestAspect").cast<TestAspect>();
    assert(fileTest);
    
    assert(fileEntity->getVar<bool>("loadTest_called",false) == true);
    
    return 0;
}
