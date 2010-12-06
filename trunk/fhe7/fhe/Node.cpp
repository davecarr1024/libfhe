#include <fhe/Node.h>
#include <fhe/NodeFactory.h>

namespace boost
{
    
    void intrusive_ptr_add_ref( fhe::Node* node )
    {
        printf( "inc\n" );
        if ( !node->m_refs )
        {
            fhe::NodeFactory::instance().init( node );
        }
        node->m_refs++;
    }
    
    void intrusive_ptr_release( fhe::Node* node )
    {
        printf( "dec\n" );
        if ( !--node->m_refs )
        {
            printf( "del\n");
            delete node;
        }
    }
    
}

namespace fhe
{
    
    Node::Node() :
        m_refs( 0 ),
        m_parent( 0 )
    {
    }
    
    Node::~Node()
    {
    }
    
    void Node::defineClass()
    {
        boost::python::class_<Node, boost::noncopyable>( "Node", boost::python::no_init )
            ;
    }
    
    NodePtr Node::parent() const
    {
        return m_parent;
    }
    
    NodePtr Node::root() const
    {
        return m_parent ? m_parent->root() : const_cast<Node*>( this );
    }
    
    bool Node::hasChild( const NodePtr& child ) const
    {
        return m_children.find( child ) != m_children.end();
    }
    
    Node::ChildrenIterator Node::childrenBegin() const
    {
        return m_children.begin();
    }
    
    Node::ChildrenIterator Node::childrenEnd() const
    {
        return m_children.end();
    }
    
    void Node::attachToParent( const NodePtr& parent )
    {
        if ( parent.get() != m_parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
                m_parent->attachChild( this );
            }
        }
    }
    
    void Node::detachFromParent()
    {
        if ( m_parent )
        {
            Node* parent = m_parent;
            m_parent = 0;
            parent->detachChild( this );
        }
    }
    
    void Node::attachChild( const NodePtr& child )
    {
        if ( child && !hasChild( child ) )
        {
            m_children.insert( child );
            child->attachToParent( this );
        }
    }
    
    void Node::detachChild( const NodePtr& child )
    {
        if ( child && hasChild( child ) )
        {
            m_children.erase( child );
            child->detachFromParent();
        }
    }
    
    Val Node::call( const std::string& name, const std::vector< Val >& args )
    {
        std::map< std::string, IFuncPtr >::iterator i = m_funcs.find( name );
        FHE_ASSERT_MSG( i != m_funcs.end(), "unable to call unknown func %s", name.c_str() );
        return i->second->call( args );
    }
    
    bool Node::tryCall( const std::string& name, const std::vector< Val >& args, Val& ret )
    {
        std::map< std::string, IFuncPtr >::iterator i = m_funcs.find( name );
        return i != m_funcs.end() ? i->second->tryCall( args, ret ) : false;
    }
    
    void Node::publish( const std::string& name, const std::vector< Val >& args )
    {
        call( name, args );
        for ( ChildrenIterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            (*i)->publish( name, args );
        }
    }
    
    void Node::addFunc( const IFuncPtr& func )
    {
        m_funcs[func->name()] = func;
    }
    
    void Node::addVar( const IVarPtr& var )
    {
        m_vars[var->name()] = var;
    }
    
    bool Node::hasVar( const std::string& name ) const
    {
        return m_vars.find( name ) != m_vars.end();
    }
    
    bool Node::tryGetVar( const std::string& name, Val& v ) const
    {
        std::map< std::string, IVarPtr >::const_iterator i = m_vars.find( name );
        if ( i != m_vars.end() )
        {
            v = i->second->get();
            return true;
        }
        else
        {
            return false;
        }
    }
    
    bool Node::getAncestorVar( const std::string& name, Val& v ) const
    {
        if ( tryGetVar( name, v ) )
        {
            return true;
        }
        else if ( m_parent )
        {
            return m_parent->getAncestorVar( name, v );
        }
        else
        {
            return false;
        }
    }
    
    Val Node::getVar( const std::string& name ) const
    {
        std::map< std::string, IVarPtr >::const_iterator i = m_vars.find( name );
        FHE_ASSERT_MSG( i != m_vars.end(), "unable to get unknown var %s", name.c_str() );
        return i->second->get();
    }
    
    void Node::setVar( const std::string& name, const Val& v )
    {
        std::map< std::string, IVarPtr >::iterator i = m_vars.find( name );
        FHE_ASSERT_MSG( i != m_vars.end(), "unable to set unknown var %s", name.c_str() );
        i->second->set( v );
    }
    
    bool Node::trySetVar( const std::string& name, const Val& v )
    {
        std::map< std::string, IVarPtr >::iterator i = m_vars.find( name );
        return i != m_vars.end() ? i->second->trySet( v ) : false;
    }
    
    INodeIntDesc::INodeIntDesc( const std::string& name ) :
        m_name( name )
    {
    }
    
    INodeDesc::INodeDesc( const std::string& name ) :
        INodeIntDesc( name )
    {
    }
    
    void INodeIntDesc::addFunc( const IFuncDescPtr& func )
    {
        m_funcs.push_back( func );
    }
    
    void INodeIntDesc::addVar( const IVarDescPtr& var )
    {
        m_vars.push_back( var );
    }
    
    void INodeIntDesc::init( Node* node ) const
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
    
    std::string INodeIntDesc::name() const
    {
        return m_name;
    }
    
    void INodeIntDesc::addDep( const INodeIntDescPtr& dep )
    {
        m_deps.push_back( dep );
    }
    
    bool INodeIntDesc::isDep( const INodeIntDescPtr& dep ) const
    {
        if ( dep.get() == this )
        {
            return true;
        }
        
        for ( std::vector< INodeIntDescPtr >::const_iterator i = m_deps.begin(); i != m_deps.end(); ++i )
        {
            if ( (*i)->isDep( dep ) )
            {
                return true;
            }
        }
        
        return false;
    }
    
}
