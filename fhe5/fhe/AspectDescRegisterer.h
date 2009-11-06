#ifndef ASPECT_DESC_REGISTERER_H
#define ASPECT_DESC_REGISTERER_H

#include "AspectFactory.h"

namespace fhe
{
    
    template <class T>
    class AspectDescRegisterer
    {
        public:
            AspectDescRegisterer( const std::string& name, const std::string& parent )
            {
                AspectFactory::instance().addDesc(new AspectDesc<T>(name,parent) );
            }
    };
    
    #define FHE_ASPECT(className,parentName) \
        AspectDescRegisterer<className> className##_registerer(#className,#parentName);
    
}

#endif
