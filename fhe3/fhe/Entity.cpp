#include "Entity.h"

namespace fhe
{
    EntityPtr Entity::root( new Entity("root") );
    
    Entity::Entity( const std::string& name ) :
        m_parent(0),
        m_name(name)
    {
    }
    
    Entity::~Entity()
    {
    }
    
    void Entity::attachToParent( EntityPtr parent )
    {
        if ( parent != m_parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
                m_parent->addChild(EntityPtr(this,true));
            }
        }
    }
    
    void Entity::detachFromParent()
    {
        if ( m_parent )
        {
            Entity* parent = m_parent;
            m_parent = 0;
            parent->removeChild(EntityPtr(this,true));
        }
    }
    
    void Entity::addChild( EntityPtr child )
    {
        if ( child && !hasChild(child->m_name) )
        {
            m_children[child->m_name] = child;
            child->attachToParent(EntityPtr(this,true));
        }
    }
    
    void Entity::removeChild( EntityPtr child )
    {
        if ( child && hasChild(child->m_name) )
        {
            m_children.erase(child->m_name);
            child->detachFromParent();
        }
    }
    
    bool Entity::hasChild( const std::string& name )
    {
        return m_children.find(name) != m_children.end();
    }
    
    EntityPtr Entity::getChild( const std::string& name )
    {
        EntityMap::iterator i = m_children.find(name);
        return i == m_children.end() ? 0 : i->second;
    }
    
    EntityPtr Entity::getParent()
    {
        return EntityPtr(m_parent,true);
    }
}
