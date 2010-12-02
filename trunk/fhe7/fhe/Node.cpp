#include <fhe/Node.h>

namespace fhe
{
    
    Node::Node()
    {
    }
    
    Node::~Node()
    {
    }
    
    Val Node::call( const std::string& name, const std::vector< Val >& args, const Val& def )
    {
        std::map< std::string, IFuncPtr >::iterator i = m_funcs.find( name );
        return i != m_funcs.end() ? i->second->call( args, def ) : def;
    }
    
    INodeDesc::INodeDesc( const std::string& name ) :
        m_name( name )
    {
    }
    
    void INodeDesc::addFunc( const IFuncDescPtr& func )
    {
        m_funcs.push_back( func );
    }
    
    void INodeDesc::init( NodePtr& node )
    {
        for ( std::vector< IFuncDescPtr >::const_iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            node->addFunc( (*i)->build( node.get() ) );
        }
    }
    
    std::string INodeDesc::name() const
    {
        return m_name;
    }
    
}
