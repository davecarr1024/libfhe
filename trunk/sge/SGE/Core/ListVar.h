#ifndef LIST_VAR_H
#define LIST_VAR_H

#include "Var.h"

namespace SGE
{
    
    class ListVar : public Var
    {
        private:
            VarList m_list;
            
        public:
            ListVar();
            
            bool empty() const;
            int size() const;
            VarPtr get( int pos );
            void append( VarPtr var );
            void remove( VarPtr var );
            bool contains( VarPtr var ) const;
            void clear();
    };

    typedef Poco::AutoPtr<ListVar> ListVarPtr;
}

#endif
