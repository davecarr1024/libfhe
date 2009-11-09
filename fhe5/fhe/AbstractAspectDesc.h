#ifndef ABSTRACT_ASPECT_DESC_H
#define ABSTRACT_ASPECT_DESC_H

#include "Aspect.h"
#include "FuncDesc.h"
#include <map>


namespace fhe
{
    class AbstractAspectDesc
    {
        public:
            virtual const std::type_info& getType()=0;
            
            virtual std::string getName()=0;
            
            virtual Aspect* build()=0;
            
            virtual void init( Aspect* aspect )=0;
            
            virtual void addFunc( AbstractFuncDesc* func )=0;
            
            template <class T>
            bool is()
            {
                return typeid(T) == getType();
            }
    };
    
}

#endif

