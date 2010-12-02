#ifndef FHE_FUNCDESC_H
#define FHE_FUNCDESC_H

#include <fhe/IFuncDesc.h>
#include <fhe/Func.h>

namespace fhe
{
    template <class TObj, class TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, class TArg )>
    class FuncDesc;
    
    #define FUNCDESC_id( z, n, data ) data

    #define FUNCDESC_iter( z, n, unused ) \
        template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class FuncDesc<TObj, TRet, BOOST_PP_ENUM_PARAMS( n, TArg ) \
                             BOOST_PP_COMMA_IF( n ) \
                             BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNCDESC_id, void ) \
                             > : public IFuncDesc { \
            public: \
                typedef TRet (TObj::*Ptr)( BOOST_PP_ENUM_PARAMS( n, TArg ) ); \
                FuncDesc( const std::string& name, Ptr ptr ) : m_name( name ), m_ptr( ptr ) {} \
                IFuncPtr build( Node* node ) { \
                    return IFuncPtr( new Func<TObj, TRet, BOOST_PP_ENUM_PARAMS( n, TArg ) \
                             BOOST_PP_COMMA_IF( n ) \
                             BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNCDESC_id, void ) \
                             >( m_name, node, m_ptr ) ); \
                } \
            private: \
                std::string m_name; \
                Ptr m_ptr; \
        }; \
        
    BOOST_PP_REPEAT( FHE_ARGS, FUNCDESC_iter, ~ )
    
    #undef FUNCDESC_iter
    #undef FUNCDESC_id

}

#endif
