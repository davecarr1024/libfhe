#include <fhe/Entity.h>
#include <cassert>

using namespace fhe;

class TestAspect : public Aspect
{
    public:
        TestAspect( const std::string& name ) : Aspect(name) {}
};

FHE_ASPECT(TestAspect);

int main()
{
    Entity test("test");
    
    AspectPtr a = test.addAspect("TestAspect");
    assert(a);
    
    a->runScript("test/test.py");
    
    return 0;
}
