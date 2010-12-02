#include <fhe/NodeFactory.h>

namespace fhe
{
    
    NodeFactory::NodeFactory()
    {
    }
    
    NodeFactory::~NodeFactory()
    {
    }
    
    NodeFactory& NodeFactory::instance()
    {
        static NodeFactory nf;
        return nf;
    }
    
    void NodeFactory::addNode( const INodeDescPtr& node )
    {
        m_nodes.push_back( node );
    }
    
    INodeDescPtr NodeFactory::getNode( const std::string& name ) const
    {
        for ( std::vector< INodeDescPtr >::const_iterator i = m_nodes.begin(); i != m_nodes.end(); ++i )
        {
            if ( (*i)->name() == name )
            {
                return *i;
            }
        }
        return INodeDescPtr();
    }
    
    NodePtr NodeFactory::build( const std::string& name ) const
    {
        if ( INodeDescPtr desc = getNode( name ) )
        {
            return desc->build();
        }
        else
        {
            return NodePtr();
        }
    }
    
    void NodeFactory::init( Node* node ) const
    {
        for ( std::vector< INodeDescPtr >::const_iterator i = m_nodes.begin(); i != m_nodes.end(); ++i )
        {
            (*i)->init( node );
        }
    }
}
