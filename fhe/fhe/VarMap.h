#ifndef VARMAP_H
#define VARMAP_H

#include "Var.h"

#include <vector>
#include <map>
#include <string>

namespace fhe
{
    
    class VarMap
    {
        private:
            std::map<std::string, IVarWrapper*> m_vars;
            
            std::string getType( boost::python::object obj );
            
        public:
            VarMap();
            VarMap( const VarMap& val );
            
            VarMap( boost::python::object obj );
            
            VarMap& operator=( const VarMap& val );
            
            virtual ~VarMap();
            
            void clone( const VarMap& val );
            
            void removeVar( const std::string& name );
            
            void clearVars();
            
            std::vector<std::string> getVarNames();
            
            bool pyHasVar( const std::string& name );
            bool pyCanSetVar( const std::string& name, boost::python::object val );
            void pySetVar( const std::string& name, boost::python::object val );
            boost::python::object pyGetVar( const std::string& name );
            boost::python::object pyGetVarDef( const std::string& name, boost::python::object def );
            boost::python::object pyGetVarNames();
            
            template <class T>
            bool hasVar( const std::string& name );

            template <class T>
            bool canSetVar( const std::string& name );
            
            template <class T>
            bool canGetVar( const std::string& name );
            
            template <class T>
            void setVar( const std::string& name, const T& val );

            template <class T>
            void connectVar( const std::string& name, IVar<T>* val );

            template <class T>
            T getVar( const std::string& name );

            template <class T>
            T getVar( const std::string& name, T def );

            boost::python::object toPy();
            
            static VarMap fromPy( boost::python::object obj );
            
            static boost::python::object defineClass();
    };
    
}

#endif
