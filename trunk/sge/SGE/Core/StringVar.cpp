#include "StringVar.h"

namespace SGE
{
    StringVar::StringVar( const std::string& val ) :
        Var( STRING ),
        m_val( val )
    {
    }
    
    void StringVar::set( const std::string& val )
    {
        m_val = val;
    }
    
    std::string StringVar::get() const
    {
        return m_val;
    }
}
