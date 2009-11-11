#include <fhe/Entity.h>
using namespace fhe;

int main()
{
    EntityPtr root(new Entity("root") );
    assert(root->loadChild("Python/test.app")->call("foo").get<std::string>("") == "bar");
    return 0;
}
