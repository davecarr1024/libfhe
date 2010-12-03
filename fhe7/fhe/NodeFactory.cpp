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
        FHE_ASSERT( node );
        FHE_ASSERT_MSG( m_nodes.find( node->name() ) == m_nodes.end(), 
                        "unable to add node type %s: duplicates previous type", node->name().c_str() );
        FHE_ASSERT_MSG( m_nodeInts.find( node->name() ) == m_nodeInts.end(),
                        "unable to add node type %s: duplicates previous int", node->name().c_str() );
        m_nodes[node->name()] = node;
    }
    
    bool NodeFactory::hasNode( const std::string& name ) const
    {
        return m_nodes.find( name ) != m_nodes.end();
    }
    
    INodeDescPtr NodeFactory::getNode( const std::string& name ) const
    {
        std::map< std::string, INodeDescPtr >::const_iterator i = m_nodes.find( name );
        FHE_ASSERT_MSG( i != m_nodes.end(), "unable to get unknown node type %s", name.c_str() );
        return i->second;
    }
    
    void NodeFactory::addNodeInt( const INodeIntDescPtr& nodeInt )
    {
        FHE_ASSERT( nodeInt );
        FHE_ASSERT_MSG( m_nodeInts.find( nodeInt->name() ) == m_nodeInts.end(),
                        "unable to add node int %s: duplicates previous int", nodeInt->name().c_str() );
        FHE_ASSERT_MSG( m_nodes.find( nodeInt->name() ) == m_nodes.end(),
                        "unable to add node int %s: duplicates previous node", nodeInt->name().c_str() );
        m_nodeInts[nodeInt->name()] = nodeInt;
    }
    
    bool NodeFactory::hasNodeInt( const std::string& name ) const
    {
        return m_nodeInts.find( name ) != m_nodeInts.end();
    }
    
    INodeIntDescPtr NodeFactory::getNodeInt( const std::string& name ) const
    {
        std::map< std::string, INodeIntDescPtr >::const_iterator i = m_nodeInts.find( name );
        FHE_ASSERT_MSG( i != m_nodeInts.end(), "unable to get unknown node int %s", name.c_str() );
        return i->second;
    }
    
    NodeFactory& NodeFactory::instance()
    {
        static NodeFactory nf;
        return nf;
    }
    
    NodePtr NodeFactory::build( const std::string& name ) const
    {
        FHE_ASSERT_MSG( hasNode( name ), "unable to build unknown node type %s", name.c_str() );
        return NodePtr( getNode( name )->build() );
    }
    
    void NodeFactory::init( Node* node ) const
    {
        std::vector< INodeIntDescPtr > descs;
        for ( std::map< std::string, INodeDescPtr >::const_iterator i = m_nodes.begin(); i != m_nodes.end(); ++i )
        {
            if ( i->second->canInit( node ) )
            {
                descs.push_back( i->second );
            }
        }
        for ( std::map< std::string, INodeIntDescPtr >::const_iterator i = m_nodeInts.begin(); i != m_nodeInts.end(); ++i )
        {
            if ( i->second->canInit( node ) )
            {
                descs.push_back( i->second );
            }
        }
        
        while ( !descs.empty() )
        {
            bool found = false;
            for ( std::vector< INodeIntDescPtr >::iterator i = descs.begin(); !found && i != descs.end(); ++i )
            {
                bool anyDeps = false;
                for ( std::vector< INodeIntDescPtr >::const_iterator j = descs.begin(); !anyDeps && j != descs.end(); ++j )
                {
                    if ( *i != *j && (*i)->isDep( *j ) )
                    {
                        anyDeps = true;
                    }
                }
                if ( !anyDeps )
                {
                    (*i)->init( node );
                    descs.erase( i );
                    found = true;
                }
            }
            FHE_ASSERT_MSG( found, "unable to initialize node due to cyclic dependency" );
        }
    }
    
}
