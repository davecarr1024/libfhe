#include "Aspect.h"
#include "Entity.h"

namespace gge
{

    GGE_ASPECT(Aspect);

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
    
    Entity* Aspect::getEntity()
    {
        return m_entity;
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
                if ( hasFunc<void,void>("on_attach") )
                {
                    call<void>("on_attach");
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
            if ( hasFunc<void,void>("on_detach") )
            {
                call<void>("on_detach");
            }
        }
    }
}
