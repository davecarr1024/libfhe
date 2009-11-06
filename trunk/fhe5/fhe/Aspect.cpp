#include "Aspect.h"
#include "Entity.h"

namespace fhe
{
    AspectDescRegisterer<Aspect> Aspect_registerer("Aspect","");
    
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
    
}
