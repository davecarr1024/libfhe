#include "Aspect.h"

#include <cstdarg>

namespace gge
{
    
    Aspect::Aspect() :
        m_entity(0)
    {
    }
    
    Aspect::~Aspect()
    {
    }
    
    void Aspect::init( const std::string& name )
    {
        m_name = name;
    }
    
    std::string Aspect::getName()
    {
        return m_name;
    }
    
    std::string Aspect::getPath()
    {
        return (m_entity ? m_entity->getName() : "<no entity>") + "." + m_name;
    }
    
    Entity* Aspect::getEntity()
    {
        return m_entity;
    }
    
    void Aspect::log( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        printf("%s: %s\n", getPath().c_str(), buffer);
    }

    void Aspect::error( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        throw std::runtime_error( getPath() + ": ERROR: " + buffer );
    }

    void Aspect::attachToEntity( Entity* entity )
    {
        if ( entity != m_entity )
        {
            detachFromEntity();
            m_entity = entity;
            if ( m_entity )
            {
                m_entity->addAspect(this);
                if ( hasFunc("on_attach") )
                {
                    call("on_attach");
                }
            }
        }
    }
    
    void Aspect::detachFromEntity()
    {
        if ( m_entity )
        {
            Entity* entity = m_entity;
            m_entity = 0;
            entity->removeAspect(this);
            if ( hasFunc("on_detach") )
            {
                call("on_detach");
            }
        }
    }
    
    void Aspect::loadData( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            std::string load = std::string("load_") + e->Value();
            if ( hasFunc(load) )
            {
                call(load,Var::build<TiXmlHandle>(e));
            }
        }
    }
}
