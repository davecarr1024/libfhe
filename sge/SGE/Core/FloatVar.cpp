#include "FloatVar.h"

namespace SGE
{
    FloatVar::FloatVar( float val ) :
        Var( FLOAT ),
        m_val( val )
    {
    }
    
    void FloatVar::set( float val )
    {
        m_val = val;
    }
    
    float FloatVar::get() const
    {
        return m_val;
    }
}
