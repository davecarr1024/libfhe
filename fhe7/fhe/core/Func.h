#ifndef FHE_FUNC_H
#define FHE_FUNC_H

#include <fhe/core/Val.h>

#include <boost/shared_ptr.hpp>
#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/arithmetic/sub.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>

#include <vector>

#ifndef FHE_ARGS
#define FHE_ARGS 3
#endif

namespace fhe
{

    class IFunc
    {
        public:
            virtual Val call( const std::vector< Val >& args ) = 0;
            virtual bool tryCall( const std::vector< Val >& args, Val& ret ) = 0;
            
            virtual std::string name() const = 0;
    };
    
    typedef boost::shared_ptr< IFunc > IFuncPtr;
    
    template <class TObj, class TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, class TArg )>
    class Func;
    
    #define FUNC_id( z, n, data ) data
    
    #define FUNC_typecheck( z, n, unused ) FHE_ASSERT_MSG( args[n].is<BOOST_PP_CAT( TArg, n )>(), \
        "arg %d to %s::%s type mismatch: expected %s got %s", \
        n, m_nodeType.c_str(), m_name.c_str(), typeid( BOOST_PP_CAT( TArg, n ) ).name(), args[n].type().c_str() );
        
    #define FUNC_typeif( z, n, unused ) if ( args[n].is<BOOST_PP_CAT( TArg, n )>() )
    
    #define FUNC_arg( z, n, unused ) args[n].get< BOOST_PP_CAT( TArg, n ) >()
    
    #define FUNC_iter( z, n, unused ) \
        template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class Func<TObj,void, BOOST_PP_ENUM_PARAMS( n, TArg ) BOOST_PP_COMMA_IF( n ) \
                    BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNC_id, void ) > \
            : public IFunc { \
            public: \
                typedef void (TObj::*Ptr)( BOOST_PP_ENUM_PARAMS( n, TArg ) ); \
                Func( const std::string& nodeType, const std::string& name, TObj* obj, Ptr ptr ) \
                    : m_nodeType( nodeType ), m_name( name ), m_obj( obj ), m_ptr( ptr ) {} \
                Val call( const std::vector< Val >& args ) { \
                    FHE_ASSERT_MSG( args.size() == n, \
                        "wrong number of arguments to %s::%s: got %d expected %d", \
                        m_nodeType.c_str(), m_name.c_str(), args.size(), n ); \
                    BOOST_PP_REPEAT( n, FUNC_typecheck, ~ ) \
                    (m_obj->*m_ptr)( BOOST_PP_ENUM( n, FUNC_arg, ~ ) ); \
                    return Val(); \
                } \
                bool tryCall( const std::vector< Val >& args, Val& ret ) { \
                    if ( args.size() == n ) BOOST_PP_REPEAT( n, FUNC_typeif, ~ ) {\
                        (m_obj->*m_ptr)( BOOST_PP_ENUM( n, FUNC_arg, ~ ) ); \
                        return true; \
                    } \
                    return false; \
                } \
                std::string name() const { return m_name; } \
            private: \
                std::string m_nodeType; \
                std::string m_name; \
                TObj* m_obj; \
                Ptr m_ptr; \
        };
                            
    BOOST_PP_REPEAT( FHE_ARGS, FUNC_iter, ~ )
    
    #undef FUNC_iter
    #undef FUNC_typecheck
    #undef FUNC_id
    
    #define RETFUNC_id( z, n, data ) data
    
    #define RETFUNC_typecheck( z, n, unused ) FHE_ASSERT_MSG( args[n].is<BOOST_PP_CAT( TArg, n )>(), \
        "arg %d to %s::%s type mismatch: expected %s got %s", \
        n, m_nodeType.c_str(), m_name.c_str(), typeid( BOOST_PP_CAT( TArg, n ) ).name(), args[n].type().c_str() );
    
    #define RETFUNC_typeif( z, n, unused ) if ( args[n].is<BOOST_PP_CAT( TArg, n )>() )
    
    #define RETFUNC_arg( z, n, unused ) args[n].get< BOOST_PP_CAT( TArg, n ) >()
    
    #define RETFUNC_iter( z, n, unused ) \
        template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class Func<TObj,TRet, BOOST_PP_ENUM_PARAMS( n, TArg ) BOOST_PP_COMMA_IF( n ) \
                    BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), RETFUNC_id, void ) > \
            : public IFunc { \
            public: \
                typedef TRet (TObj::*Ptr)( BOOST_PP_ENUM_PARAMS( n, TArg ) ); \
                Func( const std::string& nodeType, const std::string& name, TObj* obj, Ptr ptr ) \
                    : m_nodeType( nodeType ), m_name( name ), m_obj( obj ), m_ptr( ptr ) {} \
                Val call( const std::vector< Val >& args ) { \
                    FHE_ASSERT_MSG( args.size() == n, \
                        "wrong number of arguments to %s::%s: got %d expected %d", \
                        m_nodeType.c_str(), m_name.c_str(), args.size(), n ); \
                    BOOST_PP_REPEAT( n, RETFUNC_typecheck, ~ ) \
                    return (m_obj->*m_ptr)( BOOST_PP_ENUM( n, RETFUNC_arg, ~ ) ); \
                } \
                bool tryCall( const std::vector< Val >& args, Val& ret ) { \
                    if ( args.size() == n ) BOOST_PP_REPEAT( n, RETFUNC_typeif, ~ ) {\
                        ret = (m_obj->*m_ptr)( BOOST_PP_ENUM( n, RETFUNC_arg, ~ ) ); \
                        return true; \
                    } \
                    return false; \
                } \
                std::string name() const { return m_name; } \
            private: \
                std::string m_nodeType; \
                std::string m_name; \
                TObj* m_obj; \
                Ptr m_ptr; \
        };
                            
    BOOST_PP_REPEAT( FHE_ARGS, RETFUNC_iter, ~ )
    
    #undef RETFUNC_iter
    #undef RETFUNC_typecheck
    #undef RETFUNC_id
    
    class Node;
    
    class IFuncDesc
    {
        public:
            virtual IFuncPtr build( const std::string& nodeType, Node* node ) const = 0;
    };
    
    typedef boost::shared_ptr< IFuncDesc > IFuncDescPtr;
    
    template <class TObj, class TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, class TArg )>
    class FuncDesc;
    
    #define FUNCDESC_id( z, n, data ) data
    
    #define FUNCDESC_iter( z, n, unused ) \
        template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
        class FuncDesc<TObj,TRet, BOOST_PP_ENUM_PARAMS( n, TArg ) BOOST_PP_COMMA_IF( n ) \
            BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNCDESC_id, void )> : public IFuncDesc { \
            public: \
                typedef TRet (TObj::*Ptr)( BOOST_PP_ENUM_PARAMS( n, TArg ) ); \
                FuncDesc( const std::string& name, Ptr ptr ) : m_name( name ), m_ptr( ptr ) {} \
                IFuncPtr build( const std::string& nodeType, Node* node ) const { \
                    TObj* t = dynamic_cast<TObj*>( node ); \
                    FHE_ASSERT( t ); \
                    return IFuncPtr( new Func<TObj,TRet,BOOST_PP_ENUM_PARAMS( n, TArg ) BOOST_PP_COMMA_IF( n ) \
                        BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNCDESC_id, void )> ( nodeType, m_name, t, m_ptr ) ); \
                } \
            private: \
                std::string m_name; \
                Ptr m_ptr; \
        };
                    
    BOOST_PP_REPEAT( FHE_ARGS, FUNCDESC_iter, ~ )
    
    #undef FUNCDESC_iter
    #undef FUNCDESC_id
    
}

#endif
