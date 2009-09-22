#ifndef STRING_UTIL_H
#define STRING_UTIL_H

#include <sstream>
#include <string>
#include <vector>

namespace SGE
{
    typedef std::vector< std::string > StringList;
    
    class StringUtil
    {
        public:
            static const std::string WHITESPACE;
            
            static StringList split( const std::string& s, const std::string& delim = StringUtil::WHITESPACE );
            static std::string join( const StringList& list, const std::string& delim = " " );
            static std::string strip( const std::string& s );
            static std::string strip( const std::string& s, const std::string& delim = StringUtil::WHITESPACE );
            
            template < class T >
            static bool tryParse( const std::string& s, T& val )
            {
                std::istringstream ins(s);
                ins >> val;
                return !( ins.rdstate() & std::istringstream::failbit );
            }
    };
}

#endif
