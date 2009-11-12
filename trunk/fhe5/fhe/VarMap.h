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
            bool _hasVar( const std::string& name ) const;
            
            Var _getVar( const std::string& name ) const;
            
            void _setVar( const std::string& name, const Var& val );
            
            template <class T>
            bool hasVar( const std::string& name ) const
            {
                return _getVar(name).is<T>();
            }
            
            template <class T>
            T getVar( const std::string& name ) const
            {
                Var val =_getVar(name);
                if ( !val.is<T>() )
                {
                    throw std::runtime_error("unable to get var " + name + ", it is not the right type");
                }
                return val.get<T>();
            }
            
            template <class T>
            T getVar( const std::string& name, const T& def ) const
            {
                return _getVar(name).get<T>(def);
            }
            
            template <class T>
            void setVar( const std::string& name, const T& val )
            {
                _setVar(name,Var::build<T>(val));
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
            
            TiXmlElement* saveVars();
    };
    
}

#endif
