#ifndef FUNC_DESC_REGISTERER_H
#define FUNC_DESC_REGISTERER_H

#include "FuncDesc.h"
#include "AspectFactory.h"

namespace fhe
{
    
    template <class T>
    class FuncDescRegisterer
    {
        public:
            typedef Var (T::*Method)( const Var&);
            
            FuncDescRegisterer( const std::string& funcName, Method method, const std::string& filename )
            {
                AbstractAspectDesc* aspectDesc = AspectFactory::instance().getDesc<T>();
                if ( !aspectDesc )
                {
                    throw std::runtime_error("can't register func " + funcName + " to unknown class" );
                }
                aspectDesc->addFunc( new FuncDesc<T>(funcName,method) );
            }
    };
    
    #define FHE_FUNC_DECL(funcName) \
        Var funcName( const Var& arg );
        
    #define FHE_FUNC_IMPL(className,funcName) \
        FuncDescRegisterer<className> g_##className##_##funcName##_registerer(#funcName,&className::funcName,__FILE__); \
        Var className::funcName( const Var& arg )
}

#endif
