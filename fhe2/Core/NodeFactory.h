#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

#include "INodeBuilder.h"
#include <string>
#include <map>

namespace fhe
{
    class NodeFactory
    {
        friend class Node;
        
        private:
            std::map<std::string,INodeBuilder*> m_builders;
            
            NodeFactory();
            
        public:
            static NodeFactory& instance();
            
            bool hasBuilder( const std::string& name );
            
            void addBuilder( const std::string& name, INodeBuilder* builder );
            
            Node* buildNode( const std::string& type, const std::string& name );
    };
    
}

#endif
