#include "TestAspect.h"
using namespace gge;
using namespace gge::Test;

int main()
{
    EntityPtr app(new Entity("App"));
    
    EntityPtr entity = app->buildChild("ent");
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
    
    app->publish("msgTest",Var::build<int>(45));
    assert(entity->getVar<int>("msgTest",0) == 45);
    
    EntityPtr fileEntity = app->loadChild("Test/test.app");
    assert(fileEntity);

    assert(fileEntity->getVar<int>("i",0) == 11);
    
    AutoPtr<TestAspect> fileTest = fileEntity->getAspect("Test/TestAspect").cast<TestAspect>();
    assert(fileTest);
    
    assert(fileEntity->getVar<bool>("loadTest_called",false) == true);
    
    return 0;
}
