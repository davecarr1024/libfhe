#ifndef FUNCMAP_H
#define FUNCMAP_H

#include "Func.h"
#include <boost/python.hpp>
#include <boost/bind.hpp>
#include <string>
#include <map>

namespace fhe
{
    
    class FuncMap
    {
        private:
            std::map<std::string, AbstractFunc*> m_funcs;
            
            template <class TRet>
            void pyAddFuncWithRet( const std::string& name, boost::python::object targ, boost::python::object func );
            
            boost::python::object pyCallFuncNoArg( const std::string& name );
            
            template <class TArg>
            boost::python::object pyCallFuncWithArg( const std::string& name, const TArg& arg );
            
        public:
            FuncMap();
            ~FuncMap();
            
            void clearFuncs();
            
            void removeFunc( const std::string& name );
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name )
            {
                return m_funcs.find(name) != m_funcs.end() && m_funcs[name]->cast<TRet,TArg>();
            }
            
            void addFunc( const std::string& name, AbstractFunc* func );
            
            template <class TObj, class TRet, class TArg>
            void addFunc( const std::string& name, TRet (TObj::*method)(TArg), TObj* obj )
            {
                addFunc(name, new Func<TRet,TArg>(boost::bind(method,obj,_1)));
            }
            
            template <class TObj, class TRet>
            void addFunc( const std::string& name, TRet (TObj::*method)(), TObj* obj )
            {
                addFunc(name, new Func<TRet,void>(boost::bind(method,obj)));
            }
            
            template <class TRet, class TArg>
            TRet callFunc( const std::string& name, const TArg& arg )
            {
                bool hasThisFunc = hasFunc<TRet,TArg>(name);
                assert(hasThisFunc);
                return m_funcs[name]->cast<TRet,TArg>()->call(arg);
            }
            
            template <class TRet>
            TRet callFunc( const std::string& name )
            {
                bool hasThisFunc = hasFunc<TRet,void>(name);
                assert(hasThisFunc);
                return m_funcs[name]->cast<TRet,void>()->call();
            }
            
            bool pyHasFunc( const std::string& name );
            
            void pyAddFunc( const std::string& name, boost::python::object tret, boost::python::object targ, boost::python::object func );
            
            boost::python::object pyCallFunc( const std::string& name, boost::python::object arg );
        };
    
}

#endif
