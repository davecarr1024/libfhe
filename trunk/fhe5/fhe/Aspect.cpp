#include "Aspect.h"
#include "Entity.h"
#include <cstdarg>

namespace fhe
{
    FHE_ASPECT(Aspect,Aspect)
    
    Aspect::Aspect() :
        m_refCount(0),
        m_entity(0)
    {
    }
    
    Aspect::~Aspect()
    {
        for ( std::map<std::string,AbstractFunc*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            delete i->second;
        }
        m_funcs.clear();
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
        return std::string( m_entity ? m_entity->getPath() : "<null>" ) + "." + getName();
    }
    
    void Aspect::incRef()
    {
        ++m_refCount;
    }
    
    bool Aspect::decRef()
    {
        return --m_refCount;
    }
    
    bool Aspect::hasFunc( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end();
    }
    
    AbstractFunc* Aspect::getFunc( const std::string& name )
    {
        return hasFunc(name) ? m_funcs[name] : 0;
    }
    
    void Aspect::addFunc( AbstractFunc* func )
    {
        if ( func )
        {
            if ( hasFunc(func->getName()) )
            {
                delete m_funcs[func->getName()];
            }
            m_funcs[func->getName()] = func;
        }
    }
    
    Var Aspect::call( const std::string& name, const Var& arg )
    {
        return hasFunc(name) ? m_funcs[name]->call(arg) : Var();
    }
    
    void Aspect::attachToEntity( EntityPtr entity )
    {
        if ( entity.get() != m_entity )
        {
            detachFromEntity();
            m_entity = entity.get();
            if ( m_entity )
            {
                m_entity->addAspect(this);
                call("on_attach");
            }
        }
    }

    void Aspect::detachFromEntity()
    {
        if ( m_entity )
        {
            EntityPtr entity = m_entity;
            m_entity = 0;
            entity->removeAspect(this);
            call("on_detach");
        }
    }
    
    EntityPtr Aspect::getEntity()
    {
        return m_entity;
    }
    
    void Aspect::load( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            call(std::string("load_") + e->Value(),Var::build<TiXmlHandle>(e));
        }
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
}
