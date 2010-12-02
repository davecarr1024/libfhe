#ifndef FHE_FUNC_H
#define FHE_FUNC_H

#include <fhe/TFunc.h>

namespace fhe
{
    
    class Node;
    
    template <class TObj, class TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, class TArg )>
    class Func;
    
    #define FUNC_id( z, n, data ) data
    
    #define FUNC_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
    
    #define FUNC_iter( z, n, unused ) \
        template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class Func< TObj, void, BOOST_PP_ENUM_PARAMS( n, TArg ) \
                          BOOST_PP_COMMA_IF( n ) \
                          BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNC_id, void ) > \
            : public TFunc< void, \
                          BOOST_PP_ENUM_PARAMS( n, TArg ) \
                          BOOST_PP_COMMA_IF( n ) \
                          BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNC_id, void ) > \
        { \
            public: \
                typedef void (TObj::*Ptr)( BOOST_PP_ENUM( n, FUNC_arg, ~ ) ); \
                Func( const std::string& name, Node* node, Ptr ptr ) : m_name( name ), m_node( node ), m_ptr( ptr ) {} \
                void call( BOOST_PP_ENUM( n, FUNC_arg, ~ ) ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( m_node ) ) { \
                        (t->*m_ptr)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    } \
                } \
                std::string name() const { return m_name; } \
            private: \
                std::string m_name; \
                Node* m_node; \
                Ptr m_ptr; \
        };
        
    BOOST_PP_REPEAT( FHE_ARGS, FUNC_iter, ~ )
    
    #undef FUNC_iter
    #undef FUNC_arg
    #undef FUNC_id
    
    template <class TObj, class TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, class TArg )>
    class Func;
    
    #define RETFUNC_id( z, n, data ) data
    
    #define RETFUNC_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
    
    #define RETFUNC_iter( z, n, unused ) \
        template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class Func< TObj, TRet, BOOST_PP_ENUM_PARAMS( n, TArg ) \
                          BOOST_PP_COMMA_IF( n ) \
                          BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), RETFUNC_id, void ) > \
            : public TFunc< TRet, \
                          BOOST_PP_ENUM_PARAMS( n, TArg ) \
                          BOOST_PP_COMMA_IF( n ) \
                          BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), RETFUNC_id, void ) > \
        { \
            public: \
                typedef TRet (TObj::*Ptr)( BOOST_PP_ENUM( n, RETFUNC_arg, ~ ) ); \
                Func( const std::string& name, Node* node, Ptr ptr ) : m_name( name ), m_node( node ), m_ptr( ptr ) {} \
                TRet call( BOOST_PP_ENUM( n, RETFUNC_arg, ~ ) BOOST_PP_COMMA_IF( n ) TRet def ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( m_node ) ) { \
                        return (t->*m_ptr)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    } else { \
                        return def; \
                    } \
                } \
                std::string name() const { return m_name; } \
            private: \
                std::string m_name; \
                Node* m_node; \
                Ptr m_ptr; \
        };
        
    BOOST_PP_REPEAT( FHE_ARGS, RETFUNC_iter, ~ )
    
    #undef RETFUNC_iter
    #undef RETFUNC_arg
    #undef RETFUNC_id
    
}

#endif
