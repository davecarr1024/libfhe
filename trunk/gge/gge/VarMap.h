#ifndef VARMAP_H
#define VARMAP_H

#include "Var.h"

#include <map>
#include <string>

namespace gge
{
    
    class VarMap
    {
        private:
            std::map<std::string,Var> m_vars;
        
        public:
            VarMap();
            
            bool hasVarName( const std::string& name ) const;
            
            template <class T>
            bool hasVar( const std::string& name ) const
            {
                return (hasVarName(name) && const_cast<VarMap*>(this)->m_vars[name].is<T>()) || 
                    const_cast<VarMap*>(this)->onHasVar(name);
            }
            
            template <class T>
            T getVar( const std::string& name ) const
            {
                Var val = const_cast<VarMap*>(this)->onGetVar( name );
                if ( val.is<T>() )
                {
                    return val.get<T>();
                }
                else 
                {
                    assert(hasVar<T>(name));
                    return const_cast<VarMap*>(this)->m_vars[name].get<T>();
                }
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def ) const
            {
                Var val = const_cast<VarMap*>(this)->onGetVar( name );
                if ( val.is<T>() )
                {
                    return val.get<T>();
                }
                else if ( hasVar<T>(name) )
                {
                    return const_cast<VarMap*>(this)->m_vars[name].get<T>();
                }
                else
                {
                    return def;
                }
            }
            
            template <class T>
            void setVar( const std::string& name, const T& val )
            {
                if ( !hasVarName(name) )
                {
                    m_vars[name] = Var();
                }
                m_vars[name].set<T>(val);
                onSetVar( name, m_vars[name] );
            }
            
            virtual bool onHasVar( const std::string& name );
            virtual Var onGetVar( const std::string& name );
            virtual void onSetVar( const std::string& name, const Var& val );
    };
    
}

#endif
