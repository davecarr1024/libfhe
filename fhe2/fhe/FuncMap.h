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
            FuncMap();
            ~FuncMap();
            
            void clearFuncs();
            
            void removeFunc( const std::string& name );
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name );
            
            void addFunc( const std::string& name, AbstractFunc* func );
            
            template <class TObj, class TRet, class TArg>
            void addFunc( const std::string& name, TRet (TObj::*method)(TArg), TObj* obj );
            
            template <class TObj, class TRet>
            void addFunc( const std::string& name, TRet (TObj::*method)(), TObj* obj );
            
            template <class TRet, class TArg>
            TRet call( const std::string& name, const TArg& arg );
            
            template <class TRet>
            TRet call( const std::string& name );
            
            void pyAddFunc( const std::string& name, boost::python::object tret, 
                boost::python::object targ, boost::python::object func );
                
            boost::python::object pyCall( const std::string& name, boost::python::object arg );
            
            bool pyHasFunc( const std::string& name, boost::python::object tret, boost::python::object targ );
            
            static boost::python::object defineClass();
    };
    
}

#endif
