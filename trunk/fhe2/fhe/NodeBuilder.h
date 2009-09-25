#ifndef NODEBUILDER_H
#define NODEBUILDER_H

#include "INodeBuilder.h"
#include "NodeFactory.h"

#include <string>

namespace fhe
{
    template <class T>
    class NodeBuilder : public INodeBuilder
    {
        public:
            NodeBuilder( const std::string& name )
            {
                NodeFactory::instance().addBuilder(name,this);
            }
            
            Node* buildNode()
            {
                return new T;
            }
    };
    
    #define FHE_NODE_IMPL(type) NodeBuilder<type> g_##type##_builder(#type);
}

#endif
