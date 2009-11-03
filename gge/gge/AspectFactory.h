#ifndef ASPECTFACTORY_H
#define ASPECTFACTORY_H

#include "Aspect.h"

namespace gge
{
    template <class T>
    class AspectBuilder;
    
    class IAspectBuilder
    {
        public:
            virtual const std::type_info& getType()=0;
            virtual std::string getName()=0;
            virtual AspectPtr build()=0;
            
            template <class T>
            bool is()
            {
                return typeid(T) == getType();
            }
    };
    
    template <class T>
    class AspectBuilder : public IAspectBuilder
    {
        private:
            std::string m_name;
            
        public:
            AspectBuilder( const std::string& name, const std::string& file )
            {
                m_name = file.substr(0,file.rfind('/')+1) + name;
            }
            
            AspectPtr build()
            {
                AspectPtr aspect(new T);
                aspect->init(m_name);
                return aspect;
            }
            
            const std::type_info& getType()
            {
                return typeid(T);
            }
            
            std::string getName()
            {
                return m_name;
            }
    };
    
    class AspectFactory
    {
        private:
            AspectFactory();
            
            std::map<std::string,IAspectBuilder*> m_builders;
            
            bool hasBuilder( const std::string& name );
            
        public:
            ~AspectFactory();
            
            static AspectFactory& instance();
            
            AspectPtr build( const std::string& name );
            
            void addBuilder( IAspectBuilder* builder );
    };
    
    class AspectBuilderReg
    {
        public:
            AspectBuilderReg( IAspectBuilder* builder )
            {
                AspectFactory::instance().addBuilder(builder);
            }
    };
    
    #define GGE_ASPECT( name ) AspectBuilderReg g_##name##_builder( new AspectBuilder<name>(#name,__FILE__) );
    
}

#endif
