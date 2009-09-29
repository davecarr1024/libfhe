#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

#include "INodeBuilder.h"
#include <string>
#include <map>
#include <boost/intrusive_ptr.hpp>

namespace fhe
{
    class NodeFactory
    {
        friend class Node;
        typedef boost::intrusive_ptr<Node> NodePtr;
        
        private:
            std::map<std::string,INodeBuilder*> m_builders;
            
            NodeFactory();
            
        public:
            static NodeFactory& instance();
            
            bool hasBuilder( const std::string& name );
            
            void addBuilder( const std::string& name, INodeBuilder* builder );
            
            NodePtr buildNode( const std::string& type, const std::string& name );
    };
    
}

#endif
