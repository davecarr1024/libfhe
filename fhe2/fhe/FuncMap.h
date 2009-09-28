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
            void pyAddFuncWithRet( const std::string& name, boost::python::object targ, boost::python::object tret );
            
            boost::python::object pyCallWithNoArg( const std::string& name );
            
            template <class TArg>
            boost::python::object pyCallWithArg( const std::string& name, const TArg& arg );
            
            template <class TRet>
            bool pyHasFuncWithRet( const std::string& name, boost::python::object targ );
        
        public:
            class FuncClosure
            {
                private:
                    FuncMap* m_funcMap;
                    boost::python::object m_tret, m_targ;
                    
                public:
                    FuncClosure( FuncMap* funcMap, boost::python::object tret, boost::python::object targ ) :
                        m_funcMap(funcMap),
                        m_tret(tret),
                        m_targ(targ)
                    {
                    }
                    
                    void func( boost::python::object func )
                    {
                        std::string name = boost::python::extract<std::string>(func.attr("__name__"));
                        m_funcMap->pyAddFunc(name,m_tret,m_targ,func);
                    }
                    
                    static boost::python::object defineClass()
                    {
                        return boost::python::class_<FuncClosure>("FuncClosure",boost::python::no_init)
                            .def("__call__",&FuncClosure::func)
                        ;
                    }
            };

            FuncMap();
            ~FuncMap();
            
            void clearFuncs();
            
            void removeFunc( const std::string& name );
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name )
            {
/*                for ( std::map<std::string,AbstractFunc*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
                {
                    printf("%s(%p) ",i->first.c_str(),i->second);
                    assert(i->second);
                }
                printf("hasFunc %s %d %d\n",name.c_str(),m_funcs.find(name) != m_funcs.end(), m_funcs.find(name) != m_funcs.end() && m_funcs[name]->cast<TRet,TArg>());
                for ( std::map<std::string,AbstractFunc*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
                {
                    printf("%s(%p) ",i->first.c_str(),i->second);
                }
                printf("after\n");*/
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
            
            void pyAddFunc( const std::string& name, boost::python::object tret, 
                boost::python::object targ, boost::python::object func );
                
            boost::python::object pyCall( const std::string& name, boost::python::object arg );
            
            bool pyHasFunc( const std::string& name, boost::python::object tret, boost::python::object targ );
            
            static boost::python::object defineClass();

            boost::python::object func( boost::python::object tret, boost::python::object targ );
            
        };
    
}

#endif
