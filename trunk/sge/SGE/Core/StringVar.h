#ifndef STRING_VAR_H
#define STRING_VAR_H

#include "Var.h"

namespace SGE
{
    class StringVar : public Var
    {
        private:
            std::string m_val;
            
        public:
            StringVar( const std::string& val );
            
            void set( const std::string& val );
            std::string get() const;
    };
    
    typedef Poco::AutoPtr<StringVar> StringVarPtr;
}

#endif
