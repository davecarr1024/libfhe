#include <fhe/Entity.h>
#include <cassert>

using namespace fhe;

class TestAspect : public Aspect
{
};

FHE_ASPECT(TestAspect);

int main()
{
    EntityPtr test( new Entity("test"));
    
    AspectPtr a = test->addAspect("TestAspect");
    assert(a);
    
    a->runScript("test/test.py");
    
    assert(test->call<int>("foo") == -1);
    
    return 0;
}
