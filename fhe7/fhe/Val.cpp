#include <fhe/Val.h>

namespace fhe
{
    
    Val::Val() :
        m_data( 0 )
    {
    }
    
    Val::Val( const Val& v ) :
        m_data( v.m_data ? v.m_data->clone() : 0 )
    {
    }
    
    Val& Val::operator=( const Val& v )
    {
        clear();
        m_data = v.m_data ? v.m_data->clone() : 0;
        return *this;
    }
    
    Val::~Val()
    {
        clear();
    }
    
    bool Val::empty() const
    {
        return !m_data;
    }
    
    void Val::clear()
    {
        if ( m_data )
        {
            delete m_data;
            m_data = 0;
        }
    }
    
    std::string Val::type() const
    {
        return m_data ? m_data->type() : "null";
    }
    
}
