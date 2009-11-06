#include "AspectFactory.h"

namespace fhe
{
    
    AspectFactory::AspectFactory()
    {
    }
    
    AspectFactory::~AspectFactory()
    {
        for ( std::map<std::string,AbstractAspectDesc*>::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            delete i->second;
        }
        m_aspects.clear();
    }
    
    AspectFactory& AspectFactory::instance()
    {
        static AspectFactory adr;
        return adr;
    }
    
    void AspectFactory::addDesc( AbstractAspectDesc* desc )
    {
        if ( desc )
        {
            if ( hasDesc(desc->getName()) )
            {
                delete m_aspects[desc->getName()];
            }
            m_aspects[desc->getName()] = desc;
        }
    }
    
    bool AspectFactory::hasDesc( const std::string& name )
    {
        return m_aspects.find(name) != m_aspects.end();
    }
    
    AbstractAspectDesc* AspectFactory::getDesc( const std::string& name )
    {
         return hasDesc(name) ? m_aspects[name] : 0;
    }
    
    AspectPtr AspectFactory::buildAspect( const std::string& name )
    {
        AbstractAspectDesc* desc = getDesc(name);
        assert(desc);
        Aspect* aspect = desc->build();
        assert(aspect);
        aspect->init(name);
        desc->init(aspect);
        return aspect;
    }
    
}
