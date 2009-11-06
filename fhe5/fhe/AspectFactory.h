#ifndef ASPECT_FACTORY_H
#define ASPECT_FACTORY_H

#include <map>
#include "Aspect.h"

namespace fhe
{
    
    class AbstractAspectDesc;
    
    class AspectFactory
    {
        private:
            std::map<std::string,AbstractAspectDesc*> m_aspects;
            
            AspectFactory();
            
            AspectFactory( const AspectFactory& adr ) {}
            void operator=( const AspectFactory& adr ) {}
            
        public:
            ~AspectFactory();
            
            static AspectFactory& instance();
            
            void addDesc( AbstractAspectDesc* desc );
            
            bool hasDesc( const std::string& name );
            
            AbstractAspectDesc* getDesc( const std::string& name );
            
            AspectPtr buildAspect( const std::string& name );
    };
    
}

#include "AspectDesc.h"
#include "FuncDescRegisterer.h"
#include "AspectDescRegisterer.h"

#endif
