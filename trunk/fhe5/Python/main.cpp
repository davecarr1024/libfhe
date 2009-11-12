#include <fhe/Entity.h>
using namespace fhe;

int main()
{
    EntityPtr root(new Entity("root") );
    assert(root->loadChild("Python/test.app")->callNoArg<std::string>("foo") == "bar");
    return 0;
}
