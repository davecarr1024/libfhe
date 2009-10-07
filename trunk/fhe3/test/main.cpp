#include <fhe/Aspect.h>

using namespace fhe;

int main()
{
    Aspect a("test");
    a.runScript("test/test.py");
    return 0;
}
