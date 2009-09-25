#include <fhe/Node.h>

using namespace fhe;

int main()
{
    NodePtr n(NodeFactory::instance().buildNode("Node","root"));
    assert(n);
    n->runScript("test.py");
    return 0;
}
