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
            Var getRawVar( const std::string& name ) const;
            void setRawVar( const std::string& name, Var val );
            
            template <class T>
            bool hasVar( const std::string& name ) const
            {
                std::map<std::string,Var>::const_iterator i = m_vars.find(name);
                return ( i != m_vars.end() && i->second.is<T>() ) || onHasVar(name);
            }
            
            template <class T>
            T getVar( const std::string& name ) const
            {
                Var val = onGetVar( name );
                if ( val.is<T>() )
                {
                    return val.get<T>();
                }
                else 
                {
                    std::map<std::string,Var>::const_iterator i = m_vars.find(name);
                    assert(i != m_vars.end());
                    return i->second.get<T>();
                }
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def ) const
            {
                Var val = onGetVar( name );
                if ( val.is<T>() )
                {
                    return val.get<T>();
                }
                else
                {
                    std::map<std::string,Var>::const_iterator i = m_vars.find(name);
                    return i != m_vars.end() ? i->second.get<T>() : def;
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
            
            virtual bool onHasVar( const std::string& name ) const;
            virtual Var onGetVar( const std::string& name ) const;
            virtual void onSetVar( const std::string& name, const Var& val );
            
            boost::python::object toPy() const;
            static VarMap fromPy( boost::python::object obj );
    };
    
}

#endif
