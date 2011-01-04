#include <fhe/text/RenderContext.h>

namespace fhe
{
    namespace text
    {
        
        void RenderContext::clear()
        {
            m_buffer.clear();
        }
        
        void RenderContext::put( int x, int y, int z, const std::string& s )
        {
            for ( size_t i = 0; i < s.size(); ++i )
            {
                put( x + i, y, z, s[i] );
            }
        }
        
        void RenderContext::put( int x, int y, int z, char c )
        {
            std::pair< int, int > k( x, y );
            Buffer::const_iterator i = m_buffer.find( k );
            if ( i == m_buffer.end() || i->second.z < z )
            {
                Pixel p = { z, c };
                m_buffer[k] = p;
            }
        }
        
        bool RenderContext::get( int x, int y, char& c ) const
        {
            Buffer::const_iterator i = m_buffer.find( std::make_pair( x, y ) );
            if ( i != m_buffer.end() )
            {
                c = i->second.c;
                return true;
            }
            else
            {
                return false;
            }
        }
    }
}
