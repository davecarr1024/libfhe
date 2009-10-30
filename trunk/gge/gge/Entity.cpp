#include "Entity.h"
#include "App.h"

namespace gge
{
    
    Entity::Entity( const std::string& name ) :
        m_name(name),
        m_app(0)
    {
    }
    
    std::string Entity::getName()
    {
        return m_name;
    }
    
    App* Entity::getApp()
    {
        return m_app;
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
            addAspect(AspectFactory::instance().build(name));
        }
        return getAspect(name);
    }
    
    void Entity::attachToApp( App* app )
    {
        if ( m_app != app )
        {
            detachFromApp();
            m_app = app;
            if ( m_app )
            {
                m_app->addEntity(this);
            }
        }
    }
    
    void Entity::detachFromApp()
    {
        if ( m_app )
        {
            App* app = m_app;
            m_app = 0;
            app->removeEntity(this);
        }
    }
    
    Var Entity::onGetVar( const std::string& name )
    {
        std::string get = "get_" + name;
        if ( hasFunc<Var,void>(get) )
        {
            return call<Var>(get);
        }
        else
        {
            return Var();
        }
    }
    
    void Entity::onSetVar( const std::string& name, const Var& val )
    {
        callAll<void,const Var&>("set_" + name,val);
    }
    
    bool Entity::onHasVar( const std::string& name )
    {
        return hasFunc<Var,void>("get_" + name);
    }
}
