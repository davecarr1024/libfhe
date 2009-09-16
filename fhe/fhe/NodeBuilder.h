#ifndef NODE_BUILDER_H
#define NODE_BUILDER_H

#include "INodeBuilder.h"
#include "NodeFactory.h"

namespace fhe
{
    template <class T>
    class NodeBuilder : public INodeBuilder
    {
        private:
            std::string m_type;
        
        public:
            NodeBuilder( const std::string& type ) :
                m_type( type )
            {
                NodeFactory::instance().addBuilder( m_type, this );
            }
            
            Node* buildNode( const std::string& name )
            {
                return new T( name, m_type );
            }
    };
    
    #define NODE_IMPL( type ) NodeBuilder< type > g_##type##Builder( #type );
}

#endif
