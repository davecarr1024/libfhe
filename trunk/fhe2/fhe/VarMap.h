#ifndef VARMAP_H
#define VARMAP_H

#include "Var.h"

#include <map>

namespace fhe
{
    
    class VarMap
    {
        private:
            std::map<std::string, Var> m_vars;
            
        public:
            VarMap();
            VarMap( const VarMap& varMap );
            VarMap& operator=( const VarMap& varMap );
            VarMap( boost::python::object obj );
            
            void clearVars();
            
            void removeVar( const std::string& name );
            
            bool hasVarRaw( const std::string& name );
            
            Var getVarRaw( const std::string& name );
            
            template <class T>
            bool hasVar( const std::string& name )
            {
                return m_vars.find(name) != m_vars.end() && m_vars[name].is<T>();
            }
            
            template <class T>
            T getVar( const std::string& name )
            {
                Var var = onGetVar(name);
                if ( var.is<T>() )
                {
                    return var.get<T>();
                }
                else
                {
                    assert(hasVar<T>(name));
                    return m_vars[name].get<T>();
                }
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def )
            {
                Var var = onGetVar(name);
                if ( var.is<T>() )
                {
                    return var.get<T>();
                }
                else if ( hasVar<T>(name) )
                {
                    return m_vars[name].get<T>();
                }
                else
                {
                    return def;
                }
            }
            
            template <class T>
            void setVar( const std::string& name, const T& val )
            {
                if ( m_vars.find(name) == m_vars.end() )
                {
                    m_vars[name] = Var();
                }
                m_vars[name].set<T>(val);
                onSetVar(name,m_vars[name]);
            }
            
            bool pyHasVar( const std::string& name );
            
            boost::python::object pyGetVar( const std::string& name );
            
            boost::python::object pyGetVarDef( const std::string& name, boost::python::object def );
            
            void pySetVar( const std::string& name, boost::python::object val );
            
            boost::python::object toPy();
            
            static VarMap fromPy( boost::python::object obj );
            
            static boost::python::object defineClass();
            
            virtual void onSetVar( const std::string& name, const Var& val ) {}
            virtual Var onGetVar( const std::string& name ) { return Var(); }
    };
    
}

#endif
