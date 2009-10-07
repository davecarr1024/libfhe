#include "NodeFactory.h"
#include "Node.h"

namespace fhe
{
    
    NodeFactory::NodeFactory()
    {
    }
    
    NodeFactory::~NodeFactory()
    {
        m_builders.clear();
    }
    
    NodeFactory& NodeFactory::instance()
    {
        static NodeFactory instance;
        return instance;
    }
    
    NodePtr NodeFactory::buildNode( const std::string& type, const std::string& name )
    {
        if ( m_nodeTypeIds.find( type ) != m_nodeTypeIds.end() )
        {
            return m_builders[ m_nodeTypeIds[ type ] ]->buildNode( name );
        }
        return 0;
    }
    
    void NodeFactory::addBuilder( const std::string& type, INodeBuilder* builder )
    {
        if ( builder )
        {
            int id = m_builders.size();
            m_nodeTypeIds[type] = id;
            m_builders.push_back( builder );
        }
    }
}
