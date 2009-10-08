#ifndef ASPECT_FACTORY_H
#define ASPECT_FACTORY_H

#include "Aspect.h"

namespace fhe
{
    class IAspectBuilder
    {
        public:
            virtual AspectPtr buildAspect()=0;
            virtual std::string getName()=0;
    };
    
    template <class T>
    class AspectBuilder : public IAspectBuilder
    {
        private:
            std::string m_name;
        
        public:
            
            AspectBuilder( const std::string& name ) :
                m_name(name)
            {
            }
            
            AspectPtr buildAspect()
            {
                AspectPtr aspect(static_cast<Aspect*>(new T()),true);
                assert(aspect);
                aspect->init(m_name);
                return aspect;
            }
            
            std::string getName()
            {
                return m_name;
            }
    };
    
    class AspectFactory 
    {
        private:
            std::map<std::string,IAspectBuilder*> m_builders;
            
            AspectFactory();
            
        public:
            ~AspectFactory();
            
            static AspectFactory& instance();
            
            void addBuilder( IAspectBuilder* builder );
            
            AspectPtr buildAspect( const std::string& name );
    };
    
    class AspectBuilderRegisterer
    {
        public:
            AspectBuilderRegisterer( IAspectBuilder* builder )
            {
                AspectFactory::instance().addBuilder(builder);
            }
    };
    
    #define FHE_ASPECT(name) AspectBuilderRegisterer g_##name##_builderRegisterer( new AspectBuilder<name>(#name) );
}

#endif
