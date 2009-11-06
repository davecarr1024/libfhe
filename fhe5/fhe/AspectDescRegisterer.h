#ifndef ASPECT_DESC_REGISTERER_H
#define ASPECT_DESC_REGISTERER_H

#include "AspectFactory.h"

namespace fhe
{
    
    template <class T, class TParent>
    class AspectDescRegisterer
    {
        public:
            AspectDescRegisterer( const std::string& name, const std::string& filename )
            {
                std::string fullName = filename.substr(0,filename.rfind("/")+1) + name;
                AspectDesc<T>* desc = new AspectDesc<T>(fullName);
                if ( fullName != "fhe/Aspect" )
                {
                    desc->setParent(AspectFactory::instance().getDesc<TParent>());
                }
                AspectFactory::instance().addDesc(desc);
            }
    };
    
    #define FHE_ASPECT(className,parentName) \
        AspectDescRegisterer<className,parentName> className##_registerer(#className,__FILE__);
    
}

#endif