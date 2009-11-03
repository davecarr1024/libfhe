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
            T getVar( const std::string& name, const T& def, bool useOnGet = true ) const
            {
                Var val = getRawVar(name,useOnGet);
                return val.is<T>() ? val.get<T>() : def;
            }
            
            template <class T>
            void setVar( const std::string& name, const T& val )
            {
                setRawVar(name,Var::build<T>(val));
            }
            
            template <class T>
            T defaultVar( const std::string& name, const T& val, bool useOnGet = false )
            {
                setVar<T>(name,getVar<T>(name,val,useOnGet));
                return getVar<T>(name,val);
            }
            
            virtual bool onHasVar( const std::string& name ) const;
            virtual Var onGetVar( const std::string& name ) const;
            virtual void onSetVar( const std::string& name, const Var& val );
            
            boost::python::object toPy() const;
            static VarMap fromPy( boost::python::object obj );
    };
    
}

#endif
