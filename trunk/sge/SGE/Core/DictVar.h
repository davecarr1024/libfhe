#ifndef DICT_VAR_H
#define DICT_VAR_H

#include "Var.h"

#include <utility>

namespace SGE
{
    
    class DictVar : public Var
    {
        private:
            VarMap m_dict;
            
        public:
            DictVar();
            
            bool contains( const std::string& key );
            VarPtr get( const std::string& key );
            VarPtr get( const std::string& key, VarPtr def );
            void set( const std::string& key, VarPtr val );
            VarPtr setDefault( const std::string& key, VarPtr val );
            void remove( const std::string& key );
            bool empty() const;
            void clear();
            int size() const;
            std::pair<std::string, VarPtr> get( int pos );
    };

    typedef Poco::AutoPtr<DictVar> DictVarPtr;
}

#endif
