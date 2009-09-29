#include "NodeFactory.h"
#include "Node.h"
#include <cassert>

namespace fhe
{
    
    NodeFactory::NodeFactory()
    {
    }
    
    NodeFactory& NodeFactory::instance()
    {
        static NodeFactory nf;
        return nf;
    }
    
    bool NodeFactory::hasBuilder( const std::string& name )
    {
        return m_builders.find(name) != m_builders.end();
    }
    
    void NodeFactory::addBuilder( const std::string& name, INodeBuilder* builder )
    {
        assert(!hasBuilder(name));
        m_builders[name] = builder;
    }
    
    NodePtr NodeFactory::buildNode( const std::string& type, const std::string& name )
    {
        assert(hasBuilder(type));
        Node* node = m_builders[type]->buildNode();
        assert(node);
        node->init(type,name);
        return node;
    }
}
