#ifndef VARMAP_H
#define VARMAP_H

#include "Var.h"
#include <map>
#include <string>
#include <cassert>

namespace fhe
{
    
    class VarMap
    {
        private:
            std::map<std::string,Var> m_vars;
            
        public:
            VarMap();
            
            VarMap( boost::python::dict dict );
            
            void removeVar( const std::string& name );
            
            void clearVars();
            
            template <class T>
            bool hasVar( const std::string& name )
            {
                return m_vars.find(name) != m_vars.end() && m_vars[name].is<T>();
            }
            
            template <class T>
            T getVar( const std::string& name )
            {
                Var var = onGetVar(name);
                if (var.is<T>())
                {
                    m_vars[name] = var;
                }
                
                assert(hasVar<T>(name));
                return m_vars[name].get<T>();
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def )
            {
                Var var = onGetVar(name);
                if (var.is<T>())
                {
                    m_vars[name] = var;
                }
                
                if ( hasVar<T>(name) )
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
            
            template <class T>
            void defaultVar( const std::string& name, const T& val )
            {
                setVar<T>(name,getVar<T>(name,val));
            }

            boost::python::object toPy();
            
            static VarMap fromPy( boost::python::object obj );
            
            bool pyHasVar( const std::string& name );
            
            boost::python::object pyGetVar( const std::string& name );
            
            boost::python::object pyGetVarDef( const std::string& name, boost::python::object def );
            
            void pySetVar( const std::string& name, boost::python::object val );
            
            static boost::python::object defineClass();
            
            virtual Var onGetVar( const std::string& name );
            virtual void onSetVar( const std::string& name, const Var& val );
    };
    
}

#endif
