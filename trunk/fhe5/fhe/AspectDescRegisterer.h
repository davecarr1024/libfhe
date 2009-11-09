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
                AspectDesc<T,TParent>* desc = new AspectDesc<T,TParent>(fullName);
                AspectFactory::instance().addDesc(desc);
            }
    };
    
    #define FHE_ASPECT(className,parentName) \
        AspectDescRegisterer<className,parentName> g_##className##_registerer(#className,__FILE__);
    
}

#endif
