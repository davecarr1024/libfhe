#include <fhe/Node.h>
#include <fhe/NodeFactory.h>

namespace boost
{
    
    void intrusive_ptr_add_ref( fhe::Node* node )
    {
        if ( !node->m_refs )
        {
            fhe::NodeFactory::instance().init( node );
        }
        node->m_refs++;
    }
    
    void intrusive_ptr_release( fhe::Node* node )
    {
        if ( !--node->m_refs )
        {
            delete node;
        }
    }
    
}

namespace fhe
{
    
    Node::Node() :
        m_refs( 0 )
    {
    }
    
    Node::~Node()
    {
    }
    
    Val Node::call( const std::string& name, const std::vector< Val >& args )
    {
        std::map< std::string, IFuncPtr >::iterator i = m_funcs.find( name );
        FHE_ASSERT_MSG( i != m_funcs.end(), "unable to call unknown func %s", name.c_str() );
        return i->second->call( args );
    }
    
    void Node::addFunc( const IFuncPtr& func )
    {
        m_funcs[func->name()] = func;
    }
    
    void Node::addVar( const IVarPtr& var )
    {
        m_vars[var->name()] = var;
    }
    
    INodeDesc::INodeDesc( const std::string& name ) :
        m_name( name )
    {
    }
    
    void INodeDesc::addFunc( const IFuncDescPtr& func )
    {
        m_funcs.push_back( func );
    }
    
    void INodeDesc::addVar( const IVarDescPtr& var )
    {
        m_vars.push_back( var );
    }
    
    void INodeDesc::init( Node* node ) const
    {
        FHE_ASSERT( canInit( node ) );
        for ( std::vector< IFuncDescPtr >::const_iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            node->addFunc( (*i)->build( m_name, node ) );
        }
        for ( std::vector< IVarDescPtr >::const_iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            node->addVar( (*i)->build( node ) );
        }
    }
    
    std::string INodeDesc::name() const
    {
        return m_name;
    }
    
    Val Node::get( const std::string& name ) const
    {
        std::map< std::string, IVarPtr >::const_iterator i = m_vars.find( name );
        FHE_ASSERT_MSG( i != m_vars.end(), "unable to get unknown var %s", name.c_str() );
        return i->second->get();
    }
    
    void Node::set( const std::string& name, const Val& v )
    {
        std::map< std::string, IVarPtr >::iterator i = m_vars.find( name );
        FHE_ASSERT_MSG( i != m_vars.end(), "unable to set unknown var %s", name.c_str() );
        i->second->set( v );
    }
    
    void INodeDesc::addDep( const INodeDescPtr& dep )
    {
        m_deps.push_back( dep );
    }
    
    bool INodeDesc::isDep( const INodeDescPtr& dep ) const
    {
        if ( dep.get() == this )
        {
            return true;
        }
        
        for ( std::vector< INodeDescPtr >::const_iterator i = m_deps.begin(); i != m_deps.end(); ++i )
        {
            if ( (*i)->isDep( dep ) )
            {
                return true;
            }
        }
        
        return false;
    }
    
}
