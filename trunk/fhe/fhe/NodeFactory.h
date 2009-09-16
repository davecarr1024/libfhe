#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

#include "INodeBuilder.h"

#include <vector>
#include <map>
#include <string>

namespace fhe
{
    
    class Node;
    
    class NodeFactory
    {
        private:
            NodeFactory();
            ~NodeFactory();
            
            std::vector<INodeBuilder*> m_builders;
            std::map<std::string, int> m_nodeTypeIds;
            
        public:
            static NodeFactory& instance();
            
            Node* buildNode( const std::string& type, const std::string& name );
            
            void addBuilder( const std::string& type, INodeBuilder* builder );
    };
    
}

#endif
