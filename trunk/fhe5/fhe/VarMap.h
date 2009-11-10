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
            std::map<std::string,Var> m_vars;
            
        public:
            bool hasVarName( const std::string& name ) const;
            
            Var getRawVar( const std::string& name ) const;
            
            void setRawVar( const std::string& name, const Var& val );
            
            template <class T>
            bool hasVar( const std::string& name ) const
            {
                return getRawVar(name).is<T>();
            }
            
            template <class T>
            T getVar( const std::string& name ) const
            {
                return getRawVar(name).get<T>();
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def ) const
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
            
            std::vector<std::string> getVarNames() const;
            
            virtual Var onGetVar( const std::string& name ) const;
            
            virtual void onSetVar( const std::string& name, const Var& val );
            
            virtual bool onHasVar( const std::string& name ) const;
    };
    
}

#endif
