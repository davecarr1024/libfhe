#include <fhe/Node.h>

namespace boost
{
    
    void intrusive_ptr_add_ref( fhe::Node* node )
    {
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
    
    Node::Node( const std::string& name ) :
        m_refs( 0 ),
        m_name( name ),
        m_parent( 0 )
    {
    }
    
    Node::~Node()
    {
    }
    
    std::string Node::name() const
    {
        return m_name;
    }
    
    std::string Node::path() const
    {
        if ( m_parent )
        {
            if ( m_parent->m_parent )
            {
                return m_parent->path() + "/" + m_name;
            }
            else
            {
                return "/" + m_name;
            }
        }
        else
        {
            return "/";
        }
    }
    
    Node::Ptr Node::parent() const
    {
        return m_parent;
    }
    
    Node::Ptr Node::root() const
    {
        return m_parent ? m_parent->root() : const_cast<Node*>( this );
    }
    
    Node::Ptr Node::getNode( const std::string& path ) const
    {
        if ( path == "/" )
        {
            return root();
        }
        else if ( path.substr( 0, 1 ) == "/" )
        {
            return root()->getNode( path.substr( 1 ) );
        }
        else
        {
            size_t pos = path.find( "/" );
            if ( pos == std::string::npos )
            {
                return getLocalNode( path );
            }
            else 
            {
                Ptr node = getLocalNode( path.substr( 0, pos ) );
                return node ? node->getNode( path.substr( pos + 1 ) ) : Ptr();
            }
        }
    }
    
    Node::Ptr Node::getLocalNode( const std::string& path ) const
    {
        if ( path == "." )
        {
            return const_cast< Node* >( this );
        }
        else if ( path == ".." )
        {
            return m_parent;
        }
        else
        {
            return getChild( path );
        }
    }
    
    bool Node::hasChild( const std::string& name ) const
    {
        return m_children.find( name ) != m_children.end();
    }
    
    Node::Ptr Node::getChild( const std::string& name ) const
    {
        ChildIterator i = m_children.find( name );
        return i == m_children.end() ? Ptr() : i->second;
    }
    
    Node::ChildIterator Node::childBegin() const
    {
        return m_children.begin();
    }
    
    Node::ChildIterator Node::childEnd() const
    {
        return m_children.end();
    }
    
    void Node::attachToParent( const Ptr& parent )
    {
        if ( parent != m_parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
                m_parent->attachChild( this );
                onAttach();
            }
        }
    }
    
    void Node::detachFromParent()
    {
        if ( m_parent )
        {
            Ptr parent = m_parent;
            m_parent = 0;
            parent->detachChild( this );
            onDetach();
        }
    }
    
    void Node::attachChild( const Ptr& child )
    {
        if ( !hasChild( child->name() ) )
        {
            m_children[child->name()] = child;
            child->attachToParent( this );
        }
    }
    
    void Node::detachChild( const Ptr& child )
    {
        if ( hasChild( child->name() ) )
        {
            m_children.erase( child->name() );
            child->detachFromParent();
        }
    }
    
}
