#ifndef VARMAP_H
#define VARMAP_H

#include "Var.h"

#include <map>
#include <string>

namespace fhe
{
    
    class VarMap
    {
        private:
            std::map<std::string,Var> m_vars;
            
        public:
            bool hasVarName( const std::string& name );
            
            Var getRawVar( const std::string& name );
            
            void setRawVar( const std::string& name, const Var& val );
            
            template <class T>
            bool hasVar( const std::string& name )
            {
                return getRawVar(name).is<T>();
            }
            
            template <class T>
            T getVar( const std::string& name )
            {
                return getRawVar(name).get<T>();
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def )
            {
                return getRawVar(name).get<T>(def);
            }
            
            template <class T>
            void setVar( const std::string& name, const T& val )
            {
                setRawVar(name,Var::build<T>(val));
            }
            
            template <class T>
            T defaultVar( const std::string& name, const T& def )
            {
                setVar<T>(name,getVar<T>(name,def));
                return getVar<T>(name);
            }
    };
    
}

#endif
