#ifndef FHE_UTIL_H
#define FHE_UTIL_H

void fhe_assert( bool cond, const char* scond, const char* file, int line );
void fhe_error( const char* file, int line, const char* fmt, ... );
void fhe_assert_msg( bool cond, const char* scond, const char* file, int line, const char* fmt, ... );

#define FHE_ASSERT( cond ) fhe_assert( (cond), #cond, __FILE__, __LINE__ );
#define FHE_ERROR( fmt, ... ) fhe_error( __FILE__, __LINE__, fmt, ##__VA_ARGS__ );
#define FHE_ASSERT_MSG( cond, fmt, ... ) fhe_assert_msg( (cond), #cond, __FILE__, __LINE__, fmt,  ##__VA_ARGS__ );

#endif
