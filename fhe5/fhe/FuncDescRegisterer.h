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
                    throw std::runtime_error("can't register func " + funcName + " to unknown class " + className );
                }
                AspectDesc<T>* aspectDesc = abstractAspectDesc->cast<T>();
                assert(aspectDesc);
                aspectDesc->addFunc( new FuncDesc<T>(funcName,method) );
            }
    };
    
    #define FHE_FUNC(className,funcName) \
        Var funcName( const Var& arg ); \
        class funcName##_registerer { public: \
            funcName##_registerer() { \
                static bool first = true; \
                if ( first ) { \
                    first = false; \
                    FuncDescRegisterer<className>(#className,#funcName,&className::funcName,__FILE__); \
                } \
            } \
        }; \
        funcName##_registerer m_funcName##_registerer_inst;
}

#endif
