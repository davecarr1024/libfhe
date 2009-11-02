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
            
            class PyAddFunc
            {
                private:
                    FuncMap* m_funcMap;
                    boost::python::object m_tret, m_targ;
                    
                    template <class TRet>
                    void bind( boost::python::object func );
                    
                public:
                    PyAddFunc( FuncMap* funcMap, boost::python::object tret, boost::python::object targ );
                    
                    void call( boost::python::object func );
            };
            
            class PyCall
            {
                private:
                    AbstractFunc* m_func;
                    
                public:
                    PyCall( AbstractFunc* func );
                    
                    boost::python::object call( boost::python::object arg );
                    
                    boost::python::object callNoArg();
            };
            
        public:
            FuncMap();
            virtual ~FuncMap();
            
            void clearFuncs();
            
            bool hasFuncName( const std::string& name );
            
            void addFunc( const std::string& name, AbstractFunc* func );
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name ) const
            {
                std::map<std::string,AbstractFunc*>::const_iterator i = m_funcs.find(name);
                return i != m_funcs.end() && i->second->is<TRet,TArg>();
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
            void addFunc( const std::string& name, TRet (TObj::*method)(TArg), TObj* obj )
            {
                addFunc( name, new Func<TObj,TRet,TArg>(obj,method) );
            }
            
            template <class TObj, class TRet>
            void addFunc( const std::string& name, TRet (TObj::*method)(), TObj* obj )
            {
                addFunc( name, new Func<TObj,TRet,void>(obj,method) );
            }
            
            boost::python::object pyAddFunc( boost::python::object tret, boost::python::object targ );
            
            boost::python::object pyGetFunc( const std::string& name );
            
            static boost::python::object defineClass();
    };
    
}

#endif
