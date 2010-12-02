#ifndef FHE_NODE_H
#define FHE_NODE_H

#include <fhe/Var.h>
#include <fhe/Func.h>
#include <map>

namespace fhe
{
    
    class INodeDesc;
    
    class Node
    {
        private:
            friend class INodeDesc;
            
            std::map< std::string, IFuncPtr > m_funcs;
            std::map< std::string, IVarPtr > m_vars;
            
            Node( const Node& n );
            void operator=( const Node& n );
            
            void addFunc( const IFuncPtr& func );
            void addVar( const IVarPtr& var );
            
        protected:
            Node();
            
        public:
            virtual ~Node();
            
            Val get( const std::string& name, const Val& def = Val() ) const;
            void set( const std::string& name, const Val& v );
            
            template <class TObj, class TVar>
            TVar get( TVar (TObj::*ptr), TVar def = TVar() )
            {
                if ( TObj* t = dynamic_cast<TObj*>( this ) )
                {
                    return t->*ptr;
                }
                else
                {
                    return def;
                }
            }
            
            template <class TObj, class TVar>
            void set( TVar (TObj::*ptr), TVar val )
            {
                if ( TObj* t = dynamic_cast<TObj*>( this ) )
                {
                    t->*ptr = val;
                }
            }
            
            Val call( const std::string& name, const std::vector< Val >& args, const Val& def = Val() );
            
            #define CALL_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
            
            #define CALL_iter( z, n, unused ) \
                template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                void call( void (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) { \
                        (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    } \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, CALL_iter, ~ )
            
            #undef CALL_iter
            #undef CALL_arg

            #define RETCALL_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
            
            #define RETCALL_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                TRet call( TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, RETCALL_arg, ~ ), TRet def = TRet() ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) { \
                        return (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    } else { \
                        return def; \
                    } \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, RETCALL_iter, ~ )
            
            #undef RETCALL_iter
            #undef RETCALL_arg
    };
    
    typedef boost::shared_ptr< Node > NodePtr;
    
    class FuncRegisterer;
    class VarRegisterer;
    
    class INodeDesc
    {
        private:
            friend class FuncRegisterer;
            friend class VarRegisterer;
            
            std::string m_name;
            std::vector< IFuncDescPtr > m_funcs;
            std::vector< IVarDescPtr > m_vars;
            
            void addFunc( const IFuncDescPtr& func );
            void addVar( const IVarDescPtr& var );
            
        protected:
            INodeDesc( const std::string& name );
            
        public:
            void init( NodePtr& node );
            
            std::string name() const;
            
            virtual NodePtr build() const = 0;
    };
    
    typedef boost::shared_ptr< INodeDesc > INodeDescPtr;
    
    template <class T>
    class NodeDesc : public INodeDesc
    {
        public:
            NodeDesc( const std::string& name ) :
                INodeDesc( name )
            {
            }
            
            NodePtr build() const
            {
                return NodePtr( new T );
            }
    };
    
}

#endif
