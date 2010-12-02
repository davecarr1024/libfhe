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
    
    Node::Node() :
        m_refs( 0 ),
        m_parent( 0 )
    {
    }
    
    Node::~Node()
    {
    }
    
    NodePtr Node::parent() const
    {
        return m_parent;
    }
    
    NodePtr Node::root() const
    {
        return m_parent ? m_parent->root() : const_cast<Node*>( this );
    }
    
    Node::ChildIterator Node::childrenBegin() const
    {
        return m_children.begin();
    }
    
    Node::ChildIterator Node::childrenEnd() const
    {
        return m_children.end();
    }
    
    void Node::attachToParent( const NodePtr& parent )
    {
        if ( parent != m_parent )
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
            NodePtr parent = m_parent;
            m_parent = 0;
            parent->detachChild( this );
        }
    }
    
    bool Node::hasChild( const NodePtr& child )
    {
        return m_children.find( child ) != m_children.end();
    }
    
    void Node::attachChild( const NodePtr& child )
    {
        if ( !hasChild( child ) )
        {
            m_children.insert( child );
            child->attachToParent( this );
        }
    }
    
    void Node::detachChild( const NodePtr& child )
    {
        if ( hasChild( child ) )
        {
            m_children.erase( child );
            child->detachFromParent();
        }
    }
    
}
