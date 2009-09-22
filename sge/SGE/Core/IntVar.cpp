#include "IntVar.h"

namespace SGE
{
    IntVar::IntVar( int val ) :
        Var( INT ), 
        m_val(val)
    {
    }
    
    void IntVar::set( int val )
    {
        m_val = val;
    }
    
    int IntVar::get() const
    {
        return m_val;
    }
}
