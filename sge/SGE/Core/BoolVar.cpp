#include "BoolVar.h"

namespace SGE
{
    
    BoolVar::BoolVar( bool val ) :
        Var( BOOL ),
        m_val(val)
    {
    }
    
    void BoolVar::set( bool val )
    {
        m_val = val;
    }
    
    bool BoolVar::get() const
    {
        return m_val;
    }
    
}
