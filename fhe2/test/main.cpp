#include <fhe/Node.h>

using namespace fhe;

int main()
{
    NodePtr n(NodeFactory::instance().buildNode("Node","root"));
    assert(n);
/*    printf("add child\n");
    n->addChild(NodeFactory::instance().buildNode("Node","child"));*/
    printf("run script\n");
    n->runScript("test.py");
    printf("done\n");
//     n->publish("pub",VarMap());
//     assert(n->getVar<std::string>("pub") == "sub");
    return 0;
}
