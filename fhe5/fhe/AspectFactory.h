#ifndef ASPECT_FACTORY_H
#define ASPECT_FACTORY_H

#include <map>
#include "Aspect.h"
#include "AbstractAspectDesc.h"

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
            
            template <class T>
            AbstractAspectDesc* getDesc()
            {
                for ( std::map<std::string,AbstractAspectDesc*>::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->is<T>() )
                    {
                        return i->second;
                    }
                }
                return 0;
            }
    };
    
}

#include "AspectDesc.h"
#include "FuncDescRegisterer.h"
#include "AspectDescRegisterer.h"

#endif
