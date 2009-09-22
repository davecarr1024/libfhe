#include "StringUtil.h"

#include <sstream>

#include <cctype>

namespace SGE
{
    
    const std::string StringUtil::WHITESPACE = "\n\r\t ";
    
    StringList StringUtil::split( const std::string& s, const std::string& delim )
    {
        StringList toks;
        std::string tok;
        int pos = 0;
        while ( pos < s.size() )
        {
            while ( delim.find(s[pos]) != std::string::npos && pos < s.size() )
                ++pos;
            
            while ( delim.find(s[pos]) == std::string::npos && pos < s.size() )
            {
                tok += s[pos++];
            }
            
            if ( !tok.empty() )
            {
                toks.push_back(tok);
                tok = "";
            }
        }
        
        return toks;
    }
    
    std::string StringUtil::join( const StringList& sl, const std::string& delim )
    {
        std::string s;
        for (int i = 0; i < sl.size(); ++i)
        {
            s += sl[i];
            if (i < sl.size() - 1)
                s += delim;
        }
        return s;
    }
    
    std::string StringUtil::strip( const std::string& s, const std::string& delim )
    {
        int start, stop;
        
        for ( start = 0; start < s.size() && delim.find(s[start]) != std::string::npos; ++start );
        for ( stop = s.size() - 1; stop >= 0 && stop > start && delim.find(s[stop]) != std::string::npos; --stop );
        
        return s.substr(start,stop);
    }
}
