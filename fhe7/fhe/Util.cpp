#include <fhe/Util.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

void fhe_assert( bool cond, const char* scond, const char* file, int line )
{
    if ( !cond )
    {
        fprintf( stderr, "FHE_ASSERT FAILED\ncond: %s\nfile: %s\nline: %d\n", scond, file, line );
        abort();
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
        fprintf( stderr, "FHE_ASSERT FAILED\ncond: %s\nfile: %s\nline: %d\nmsg: %s\n", scond, file, line, buf );
        abort();
    }
}

void fhe_error( const char* file, int line, const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    char buf[1024];
    vsnprintf( buf, 1024, fmt, args );
    va_end( args );
    fprintf( stderr, "FHE_ERROR\nfile: %s\nline: %d\nmsg: %s\n", file, line, buf );
    abort();
}
