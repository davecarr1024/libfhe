#include <fhe/Node.h>

using namespace fhe;

int main()
{
    NodePtr n(NodeFactory::instance().buildNode("Node","root"));
    assert(n);
    n->runScript("test.py");
    n->publish("pub",VarMap());
    assert(n->getVar<std::string>("pub") == "sub");
    return 0;
}
