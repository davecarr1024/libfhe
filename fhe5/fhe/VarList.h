#ifndef VARLIST_H
#define VARLIST_H

#include "Var.h"
#include <vector>

namespace fhe
{
    
    class VarList
    {
        private:
            std::vector<Var> m_vars;
            
        public:
            int length() const;
            
            Var getRawVar( int i ) const;
            
            void appendRaw( const Var& var );
            
            template <class T>
            T getVar( int i ) const
            {
                return getRawVar(i).get<T>();
            }
            
            template <class T>
            T getVar( int i, const T& def ) const
            {
                return getRawVar(i).get<T>(def);
            }
            
            template <class T>
            void append( const T& val )
            {
                appendRaw(Var::build<T>(val));
            }
            
            void remove( int i );
    };
    
}

#endif
