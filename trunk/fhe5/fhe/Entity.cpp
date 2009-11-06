#include "Entity.h"
#include "AspectFactory.h"
#include <sstream>

namespace fhe
{
    std::map<std::string,int> Entity::m_nameCounts;
    
    Entity::Entity( const std::string& name ) :
        m_refCount(0),
        m_parent(0),
        m_name(Entity::makeName(name))
    {
    }
    
    std::string Entity::makeName( const std::string& name )
    {
        if ( Entity::m_nameCounts.find(name) == Entity::m_nameCounts.end() )
        {
            Entity::m_nameCounts[name] = 1;
            return name;
        }
        else
        {
            std::ostringstream outs;
            outs << name << "_" << ++Entity::m_nameCounts[name];
            return outs.str();
        }
    }
    
    void Entity::incRef()
    {
        ++m_refCount;
    }
    
    bool Entity::decRef()
    {
        return --m_refCount;
    }
    
    std::string Entity::getName()
    {
        return m_name;
    }
    
    std::string Entity::getPath()
    {
        if ( m_parent && m_parent->m_parent )
        {
            return m_parent->getPath() + "/" + getName();
        }
        else if ( m_parent )
        {
            return "/" + getName();
        }
        else
        {
            return "/";
        }
    }
    
    void Entity::attachToParent( EntityPtr parent )
    {
        if ( parent.get() != m_parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
                m_parent->addChild(this);
                callAll("on_attach");
            }
        }
    }
    
    void Entity::detachFromParent()
    {
        if ( m_parent )
        {
            EntityPtr parent = m_parent;
            m_parent = 0;
            parent->removeChild(this);
            callAll("on_detach");
        }
    }
    
    EntityPtr Entity::getParent()
    {
        return m_parent;
    }
    
    EntityPtr Entity::getRoot()
    {
        return m_parent ? m_parent->getRoot() : EntityPtr(this);
    }
    
    void Entity::addChild( EntityPtr child )
    {
        if ( child && !hasChild(child->getName()) )
        {
            m_children[child->getName()] = child;
            child->attachToParent(this);
        }
    }
    
    void Entity::removeChild( EntityPtr child )
    {
        if ( child && hasChild(child->getName()) )
        {
            m_children.erase(child->getName());
            child->detachFromParent();
        }
    }
    
    bool Entity::hasChild( const std::string& name )
    {
        return m_children.find(name) != m_children.end();
    }
    
    EntityPtr Entity::getChild( const std::string& name )
    {
        return hasChild(name) ? m_children[name] : 0;
    }
    
    EntityPtr Entity::buildChild( const std::string& name )
    {
        EntityPtr child( new Entity(name) );
        addChild(child);
        return child;
    }
    
    void Entity::addAspect( AspectPtr aspect )
    {
        if ( aspect && !hasAspect(aspect->getName()) )
        {
            m_aspects[aspect->getName()] = aspect;
            aspect->attachToEntity(this);
        }
    }
    
    void Entity::removeAspect( AspectPtr aspect )
    {
        if ( aspect && hasAspect(aspect->getName()) )
        {
            m_aspects.erase(aspect->getName());
            aspect->detachFromEntity();
        }
    }
    
    bool Entity::hasAspect( const std::string& name )
    {
        return m_aspects.find(name) != m_aspects.end();
    }
    
    AspectPtr Entity::getAspect( const std::string& name )
    {
        return hasAspect(name) ? m_aspects[name] : 0;
    }
    
    AspectPtr Entity::buildAspect( const std::string& name )
    {
        if ( !hasAspect(name) )
        {
            addAspect(AspectFactory::instance().buildAspect(name));
        }
        return getAspect(name);
    }
    
    bool Entity::hasFunc( const std::string& name )
    {
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                return true;
            }
        }
        return false;
    }
    
    Var Entity::call( const std::string& name, const Var& arg )
    {
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                return i->second->call(name,arg);
            }
        }
        return Var();
    }
    
    void Entity::callAll( const std::string& name, const Var& arg )
    {
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                i->second->call(name,arg);
            }
        }
    }
    
    void Entity::publish( const std::string& name, const Var& arg )
    {
//         callAll("msg_" + name,arg);
        for ( EntityMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->publish(name,arg);
        }
//         callAll("unmsg_" + name,arg);
    }
}