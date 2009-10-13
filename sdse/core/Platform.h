#ifndef PLATFORM_H
#define PLATFORM_H

namespace sdse {

#define SDSE_PLATFORM_WIN32 1
#define SDSE_PLATFORM_LINUX 2
#define SDSE_PLATFORM_APPLE 3

#define SDSE_COMPILER_MSVC 1
#define SDSE_COMPILER_GNUC 2
#define SDSE_COMPILER_BORL 3

#define SDSE_ENDIAN_LITTLE 1
#define SDSE_ENDIAN_BIG 2

#define SDSE_ARCHITECTURE_32 1
#define SDSE_ARCHITECTURE_64 2

/* Finds the compiler type and version.
*/
#if defined( _MSC_VER )
#   define SDSE_COMPILER SDSE_COMPILER_MSVC
#   define SDSE_COMP_VER _MSC_VER

#elif defined( __GNUC__ )
#   define SDSE_COMPILER SDSE_COMPILER_GNUC
#   define SDSE_COMP_VER (((__GNUC__)*100) + \
        (__GNUC_MINOR__*10) + \
        __GNUC_PATCHLEVEL__)

#elif defined( __BORLANDC__ )
#   define SDSE_COMPILER SDSE_COMPILER_BORL
#   define SDSE_COMP_VER __BCPLUSPLUS__
#   define __FUNCTION__ __FUNC__ 
#else
#   pragma error "No known compiler. Abort! Abort!"

#endif

/* See if we can use __forceinline or if we need to use __inline instead */
#if SDSE_COMPILER == SDSE_COMPILER_MSVC
#   if SDSE_COMP_VER >= 1200
#       define FORCEINLINE __forceinline
#   endif
#elif defined(__MINGW32__)
#   if !defined(FORCEINLINE)
#       define FORCEINLINE __inline
#   endif
#else
#   define FORCEINLINE __inline
#endif

/* Finds the current platform */

#if defined( __WIN32__ ) || defined( _WIN32 )
#   define SDSE_PLATFORM SDSE_PLATFORM_WIN32

#elif defined( __APPLE_CC__)
#   define SDSE_PLATFORM SDSE_PLATFORM_APPLE

#else
#   define SDSE_PLATFORM SDSE_PLATFORM_LINUX
#endif

    /* Find the arch type */
#if defined(__x86_64__) || defined(_M_X64) || defined(__powerpc64__) || defined(__alpha__) || defined(__ia64__) || defined(__s390__) || defined(__s390x__)
#   define SDSE_ARCH_TYPE SDSE_ARCHITECTURE_64
#else
#   define SDSE_ARCH_TYPE SDSE_ARCHITECTURE_32
#endif

}

#endif
