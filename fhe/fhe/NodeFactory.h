#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

#include "INodeBuilder.h"

#include <vector>
#include <map>
#include <string>
#include <boost/intrusive_ptr.hpp>

namespace fhe
{
    
    class Node;
    typedef boost::intrusive_ptr<Node> NodePtr;
    
    class NodeFactory
    {
        private:
            NodeFactory();
            ~NodeFactory();
            
            std::vector<INodeBuilder*> m_builders;
            std::map<std::string, int> m_nodeTypeIds;
            
        public:
            static NodeFactory& instance();
            
            NodePtr buildNode( const std::string& type, const std::string& name );
            
            void addBuilder( const std::string& type, INodeBuilder* builder );
    };
    
}

#endif
