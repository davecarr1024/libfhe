#ifndef FHE_TFUNC_H
#define FHE_TFUNC_H

#include <fhe/IFunc.h>

namespace fhe
{
    #define TFUNC_id( z, n, data ) data
    
    #define TFUNC_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
    
    #define TFUNC_iter( z, n, unused ) \
        template <BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class TFunc< void, BOOST_PP_ENUM_PARAMS( n, TArg ) \
                     BOOST_PP_COMMA_IF( n ) \
                     BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), TFUNC_id, void ) > : public IFunc { \
            public: virtual void call( BOOST_PP_ENUM( n, TFUNC_arg, ~ ) ) = 0; };
    
    BOOST_PP_REPEAT( FHE_ARGS, TFUNC_iter, ~ )
    
    #undef TFUNC_iter
    #undef TFUNC_id
    #undef TFUNC_arg
    
    #define TRETFUNC_id( z, n, data ) data
    
    #define TRETFUNC_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
    
    #define TRETFUNC_iter( z, n, unused ) \
        template <class TRet BOOST_PP_COMMA_IF(n) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class TFunc< TRet, \
                     BOOST_PP_ENUM_PARAMS( n, TArg ) \
                     BOOST_PP_COMMA_IF( n ) \
                     BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), TRETFUNC_id, void ) > : public IFunc { \
            public: virtual TRet call( BOOST_PP_ENUM( n, TRETFUNC_arg, ~ ) BOOST_PP_COMMA_IF(n) TRet def ) = 0; };
    
    BOOST_PP_REPEAT( FHE_ARGS, TRETFUNC_iter, ~ )
    
    #undef TRETFUNC_iter
    #undef TRETFUNC_id
    #undef TRETFUNC_arg
    
}

#endif
