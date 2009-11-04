#ifndef FUNCMAP_H
#define FUNCMAP_H

#include "Func.h"

#include <map>
#include <string>
#include <cassert>

#include <boost/bind.hpp>

namespace gge
{
    
    class FuncMap
    {
        private:
            std::map<std::string,AbstractFunc*> m_funcs;
            
        public:
            FuncMap();
            virtual ~FuncMap();
            
            void clearFuncs();
            
            bool hasFunc( const std::string& name );
            
            Var call( const std::string& name, const Var& arg = Var() );
            
            void addFunc( const std::string& name, AbstractFunc* func );
            
            template <class TObj>
            void addFunc( const std::string& name, Var (TObj::*method)(const Var&), TObj* obj )
            {
                addFunc( name, new Func<TObj>(obj,method) );
            }
            
            AbstractFunc* getFunc( const std::string& name ) const;
    };
    
}

#endif
