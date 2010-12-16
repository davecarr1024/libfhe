#include <fhe/core/Util.h>
#include <cstdio>
#include <cstdarg>
#include <stdexcept>

namespace fhe
{

    void fhe_assert( bool cond, const char* scond, const char* file, int line )
    {
        if ( !cond )
        {
            char err[1024];
            snprintf( err, 1024, "FHE_ASSERT FAILED\ncond: %s\nfile: %s\nline: %d\n", scond, file, line );
            throw std::runtime_error( err );
        }
    }

    void fhe_assert_msg( bool cond, const char* scond, const char* file, int line, const char* fmt, ... )
    {
        if ( !cond )
        {
            va_list args;
            va_start( args, fmt );
            char buf[1024];
            vsnprintf( buf, 1024, fmt, args );
            va_end( args );
            char err[1024];
            snprintf( err, 1024, "FHE_ASSERT FAILED\ncond: %s\nfile: %s\nline: %d\nmsg: %s\n", scond, file, line, buf );
            throw std::runtime_error( err );
        }
    }

    void fhe_error( const char* file, int line, const char* fmt, ... )
    {
        va_list args;
        va_start( args, fmt );
        char buf[1024];
        vsnprintf( buf, 1024, fmt, args );
        va_end( args );
        char err[1024];
        snprintf( err, 1024, "FHE_ERROR\nfile: %s\nline: %d\nmsg: %s\n", file, line, buf );
        throw std::runtime_error( err );
    }

}
