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
            Var getRawVar( const std::string& name, bool useOnGet = true ) const;
            void setRawVar( const std::string& name, Var val );
            
            template <class T>
            bool hasVar( const std::string& name ) const
            {
                std::map<std::string,Var>::const_iterator i = m_vars.find(name);
                return ( i != m_vars.end() && i->second.is<T>() ) || onHasVar(name);
            }
            
            template <class T>
            T getVar( const std::string& name) const
            {
                Var val = getRawVar(name);
                assert(val.is<T>());
                return val.get<T>();
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def ) const
            {
                Var val = getRawVar(name);
                return val.is<T>() ? val.get<T>() : def;
            }
            
            template <class T>
            void setVar( const std::string& name, const T& val )
            {
                setRawVar(name,Var::build<T>(val));
            }
            
            template <class T>
            T defaultVar( const std::string& name, const T& def )
            {
                Var val = getRawVar(name,false);
                setVar<T>( name, val.is<T>() ? val.get<T>() : def );
                return getVar<T>(name,def);
            }
            
            virtual bool onHasVar( const std::string& name ) const;
            virtual Var onGetVar( const std::string& name ) const;
            virtual void onSetVar( const std::string& name, const Var& val );
    };
    
}

#endif
