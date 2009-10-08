#ifndef FUNCMAP_H
#define FUNCMAP_H

#include "Func.h"
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

        protected:
            class PyCall
            {
                private:
                    std::string m_name;
                    AbstractFunc* m_func;
                    
                public:
                    PyCall( const std::string& name, AbstractFunc* func );
                    
                    boost::python::object call( boost::python::object arg );
                    
                    boost::python::object callNoArg();
                    
                    std::string repr();
                    
                    static boost::python::object defineClass();
            };
            
            class FuncClosure
            {
                private:
                    FuncMap* m_map;
                    boost::python::object m_tret, m_targ;
                    
                public:
                    FuncClosure( FuncMap* map, boost::python::object tret, boost::python::object targ );
                    
                    void call( boost::python::object func );
                    
                    static boost::python::object defineClass();
            };
            
        public:
            FuncMap();
            ~FuncMap();
            
            void clearFuncs();
            
            void removeFunc( const std::string& name );
            
            bool hasFuncName( const std::string& name );
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name )
            {
                return m_funcs.find(name) != m_funcs.end() && m_funcs[name]->cast<TRet,TArg>();
            }
            
            void addFunc( const std::string& name, AbstractFunc* func );
            
            template <class TObj, class TRet, class TArg>
            void addFunc( const std::string& name, TRet (TObj::*method)(TArg), TObj* obj )
            {
                addFunc(name, new Func<TObj,TRet,TArg>(obj,method));
            }
            
            template <class TObj, class TRet>
            void addFunc( const std::string& name, TRet (TObj::*method)(), TObj* obj )
            {
                addFunc(name, new Func<TObj,TRet,void>(obj,method));
            }
            
            template <class TRet, class TArg>
            void addFunc( const std::string& name, boost::function<TRet(TArg)> func )
            {
                addFunc(name, new BoostFunc<TRet,TArg>(func));
            }
            
            template <class TRet>
            void addFunc( const std::string& name, boost::function<TRet()> func )
            {
                addFunc(name, new BoostFunc<TRet,void>(func));
            }
            
            template <class TRet, class TArg>
            TRet call( const std::string& name, const TArg& arg )
            {
                bool hasThisFunc = hasFunc<TRet,TArg>(name);
                assert(hasThisFunc);
                return m_funcs[name]->cast<TRet,TArg>()->call(arg);
            }
            
            template <class TRet>
            TRet call( const std::string& name )
            {
                bool hasThisFunc = hasFunc<TRet,void>(name);
                assert(hasThisFunc);
                return m_funcs[name]->cast<TRet,void>()->call();
            }
            
            PyCall pyGetFunc( const std::string& name );
            
            void pyAddFunc( const std::string& name, boost::python::object tret, boost::python::object targ, boost::python::object func );
    };
    
}

#endif
