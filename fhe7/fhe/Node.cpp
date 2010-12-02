#include <fhe/Node.h>

namespace fhe
{
    
    Node::Node()
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
    
    void INodeDesc::init( NodePtr& node )
    {
        for ( std::vector< IFuncDescPtr >::const_iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            node->addFunc( (*i)->build( m_name, node.get() ) );
        }
        for ( std::vector< IVarDescPtr >::const_iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            node->addVar( (*i)->build( node.get() ) );
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
    
}
