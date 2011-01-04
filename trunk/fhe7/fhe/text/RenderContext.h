#ifndef FHE_TEXT_RENDER_CONTEXT_H
#define FHE_TEXT_RENDER_CONTEXT_H

#include <string>
#include <map>

namespace fhe
{
    namespace text
    {
        
        class RenderContext
        {
            private:
                struct Pixel
                {
                    int z;
                    char c;
                };

                typedef std::map< std::pair< int, int >, Pixel > Buffer;
                Buffer m_buffer;
                
            public:
                void clear();
                
                void put( int x, int y, int z, char c );
                void put( int x, int y, int z, const std::string& s );
                
                bool get( int x, int y, char& c ) const;
        };
        
    }
}

#endif
