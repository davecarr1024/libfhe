#include "App.h"

namespace gge
{
    
    App::App()
    {
    }
    
    void App::addEntity( EntityPtr entity )
    {
        if ( entity && !hasEntity(entity->getName()) )
        {
            m_entities[entity->getName()] = entity;
            entity->attachToApp(this);
        }
    }
    
    void App::removeEntity( EntityPtr entity )
    {
        if ( entity && hasEntity(entity->getName()) )
        {
            m_entities.erase(entity->getName());
            entity->detachFromApp();
        }
    }
    
    bool App::hasEntity( const std::string& name )
    {
        return m_entities.find(name) != m_entities.end();
    }
    
    EntityPtr App::getEntity( const std::string& name )
    {
        return hasEntity(name) ? m_entities[name] : 0;
    }
    
    EntityPtr App::buildEntity( const std::string& name )
    {
        EntityPtr entity( new Entity(name) );
        addEntity(entity);
        return entity;
    }
    
    void App::publish( const std::string& cmd, const VarMap& args )
    {
        std::string msg = "msg_" + cmd;
        for ( EntityMap::iterator i = m_entities.begin(); i != m_entities.end(); ++i )
        {
            i->second->callAll<void,const VarMap&>(msg,args);
        }
    }
    
}
