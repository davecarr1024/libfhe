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
            
            FuncDescRegisterer( const std::string& className, const std::string& funcName, Method method, const std::string& filename )
            {
                std::string fullClassName = filename.substr(0,filename.rfind("/")+1) + className;
                AbstractAspectDesc* abstractAspectDesc = AspectFactory::instance().getDesc(fullClassName);
                if ( !abstractAspectDesc )
                {
                    throw std::runtime_error("can't register func " + funcName + " to unknown class " + fullClassName );
                }
                AspectDesc<T>* aspectDesc = abstractAspectDesc->cast<T>();
                assert(aspectDesc);
                aspectDesc->addFunc( new FuncDesc<T>(funcName,method) );
            }
    };
    
    #define FHE_FUNC_DECL(className,funcName) \
        Var funcName( const Var& arg );
        
    #define FHE_FUNC_IMPL(className,funcName) \
        FuncDescRegisterer<className> g_##className##_##funcName##_registerer(#className,#funcName,&className::funcName,__FILE__); \
        Var className::funcName( const Var& arg )
}

#endif
