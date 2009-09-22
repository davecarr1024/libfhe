#ifndef INT_VAR_H
#define INT_VAR_H

#include "Var.h"

namespace SGE
{
    
    class IntVar : public Var
    {
        private:
            int m_val;
            
        public:
            IntVar( int val );
            
            void set( int val );
            int get() const;
    };
    
    typedef Poco::AutoPtr<IntVar> IntVarPtr;
}

#endif
