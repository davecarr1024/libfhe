#include "AspectFactory.h"

namespace gge
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
        static AspectFactory af;
        return af;
    }
    
    bool AspectFactory::hasBuilder( const std::string& name )
    {
        return m_builders.find(name) != m_builders.end();
    }
    
    AspectPtr AspectFactory::build( const std::string& name )
    {
        assert(hasBuilder(name));
        return m_builders[name]->build();
    }
    
    void AspectFactory::addBuilder( IAspectBuilder* builder )
    {
        if ( builder )
        {
            if ( hasBuilder(builder->getName()) )
            {
                delete m_builders[builder->getName()];
            }
            m_builders[builder->getName()] = builder;
        }
    }
    
}
