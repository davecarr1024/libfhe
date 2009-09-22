#ifndef BOOL_VAR_H
#define BOOL_VAR_H

#include "Var.h"

namespace SGE
{
    
    class BoolVar : public Var
    {
        private:
            bool m_val;
            
        public:
            BoolVar(bool val);
            
            void set(bool val);
            bool get() const;
    };
    
    typedef Poco::AutoPtr<BoolVar> BoolVarPtr;
}

#endif
