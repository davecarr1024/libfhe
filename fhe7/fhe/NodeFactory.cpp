#include <fhe/NodeFactory.h>

namespace fhe
{
    
    NodeFactory::NodeFactory()
    {
    }
    
    NodeFactory::~NodeFactory()
    {
    }
    
    void NodeFactory::addNode( const INodeDescPtr& node )
    {
        m_nodes[node->name()] = node;
    }
    
    INodeDescPtr NodeFactory::getNode( const std::string& name ) const
    {
        std::map< std::string, INodeDescPtr >::const_iterator i = m_nodes.find( name );
        FHE_ASSERT_MSG( i != m_nodes.end(), "unable to get unknown node type %s", name.c_str() );
        return i->second;
    }
    
    NodeFactory& NodeFactory::instance()
    {
        static NodeFactory nf;
        return nf;
    }
    
    NodePtr NodeFactory::build( const std::string& name ) const
    {
        INodeDescPtr desc = getNode( name );
        FHE_ASSERT_MSG( desc, "unable to build unknown node type %s", name.c_str() );
        return NodePtr( desc->build() );
    }
    
    void NodeFactory::init( Node* node ) const
    {
        for ( std::map< std::string, INodeDescPtr >::const_iterator i = m_nodes.begin(); i != m_nodes.end(); ++i )
        {
            if ( i->second->canInit( node ) )
            {
                i->second->init( node );
            }
        }
    }
    
}
