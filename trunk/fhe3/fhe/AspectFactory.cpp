#include "AspectFactory.h"
#include <cassert>

namespace fhe
{
    
    AspectFactory::AspectFactory()
    {
    }
    
    AspectFactory::~AspectFactory()
    {
        for ( std::map<std::string,IAspectBuilder*>::iterator i = m_builders.begin(); i != m_builders.end(); ++i )
        {
            delete i->second;
        }
        m_builders.clear();
    }
    
    AspectFactory& AspectFactory::instance()
    {
        static AspectFactory instance;
        return instance;
    }
    
    void AspectFactory::addBuilder( IAspectBuilder* builder )
    {
        assert(builder);
        assert(m_builders.find(builder->getName()) == m_builders.end());
        m_builders[builder->getName()] = builder;
    }
    
    AspectPtr AspectFactory::buildAspect( const std::string& name )
    {
        assert(m_builders.find(name) != m_builders.end());
        return m_builders[name]->buildAspect();
    }
    
}
