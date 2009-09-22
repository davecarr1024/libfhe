#ifndef FLOAT_VAR_H
#define FLOAT_VAR_H

#include "Var.h"

namespace SGE
{
    class FloatVar : public Var
    {
        private:
            float m_val;
            
        public:
            FloatVar( float val );
            
            void set( float val );
            float get() const;
    };

    typedef Poco::AutoPtr<FloatVar> FloatVarPtr;
}

#endif
