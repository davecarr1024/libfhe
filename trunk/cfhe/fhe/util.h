#ifndef FHE_UTIL_H
#define FHE_UTIL_H

void fhe_assert( int cond, const char* scond, const char* file, int line );
void fhe_assert_msg( int cond, const char* scond, const char* file, int line, const char* msg, ... );

#define FHE_ASSERT( cond ) fhe_assert( (int)(cond), #cond, __FILE__, __LINE__ );
#define FHE_ASSERT_MSG( cond, fmt, ... ) fhe_assert_msg( (int)(cond), #cond, __FILE__, __LINE__, fmt,  ##__VA_ARGS__ )

#endif
