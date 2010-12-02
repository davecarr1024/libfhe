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
        return i != m_nodes.end() ? i->second : INodeDescPtr();
    }
    
    NodeFactory& NodeFactory::instance()
    {
        static NodeFactory nf;
        return nf;
    }
    
    NodePtr NodeFactory::build( const std::string& name ) const
    {
        if ( INodeDescPtr desc = getNode( name ) )
        {
            NodePtr node = desc->build();
            desc->init( node );
            return node;
        }
        else
        {
            return NodePtr();
        }
    }
    
}
