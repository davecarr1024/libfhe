#ifndef FUNCMAP_H
#define FUNCMAP_H

#include "Func.h"

#include <map>
#include <string>
#include <cassert>

namespace gge
{
    
    class FuncMap
    {
        private:
            std::map<std::string,AbstractFunc*> m_funcs;
            
        public:
            FuncMap();
            virtual ~FuncMap();
            
            void clearFuncs();
            
            bool hasFuncName( const std::string& name );
            
            void addFunc( const std::string& name, AbstractFunc* func );
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name )
            {
                return hasFuncName(name) && m_funcs[name]->is<TRet,TArg>();
            }
            
            template <class TRet, class TArg>
            TRet call( const std::string& name, const TArg& arg )
            {
                bool hasThisFunc = hasFunc<TRet,TArg>(name);
                assert(hasThisFunc);
                return (*m_funcs[name]->cast<TRet,TArg>())(arg);
            }
            
            template <class TRet>
            TRet call( const std::string& name )
            {
                bool hasThisFunc = hasFunc<TRet,void>(name);
                assert(hasThisFunc);
                return (*m_funcs[name]->cast<TRet,void>())();
            }
            
            template <class TObj, class TRet, class TArg>
            void addFunc( const std::string& name, TObj* obj, TRet (TObj::*method)(TArg) )
            {
                addFunc( name, new Func<TObj,TRet,TArg>(obj,method) );
            }
            
            template <class TObj, class TRet>
            void addFunc( const std::string& name, TObj* obj, TRet (TObj::*method)() )
            {
                addFunc( name, new Func<TObj,TRet,void>(obj,method) );
            }
    };
    
}

#endif
