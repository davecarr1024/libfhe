#ifndef VARMAP_H
#define VARMAP_H

#include "Var.h"

#include <map>

namespace fhe
{
    
    class VarMap
    {
        protected:
            std::map<std::string, Var> m_vars;
            
        public:
            VarMap();
            VarMap( const VarMap& varMap );
            VarMap& operator=( const VarMap& varMap );
            VarMap( boost::python::object obj );
            
            void clearVars();
            
            void removeVar( const std::string& name );
            
            template <class T>
            bool hasVar( const std::string& name );
            
            template <class T>
            T getVar( const std::string& name );
            
            template <class T>
            T getVar( const std::string& name, const T& def );
            
            template <class T>
            void setVar( const std::string& name, const T& val );
            
            bool pyHasVar( const std::string& name );
            
            boost::python::object pyGetVar( const std::string& name );
            
            boost::python::object pyGetVarDef( const std::string& name, boost::python::object def );
            
            void pySetVar( const std::string& name, boost::python::object val );
            
            boost::python::object toPy();
            
            static VarMap fromPy( boost::python::object obj );
            
            static boost::python::object defineClass();
            
            virtual void onSetVar( const std::string& name, const Var& val ) {}
            virtual void onGetVar( const std::string& name ) {}
    };
    
}

#endif
