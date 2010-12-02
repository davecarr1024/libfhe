#ifndef FHE_NODE_H
#define FHE_NODE_H

#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <set>

#ifndef FHE_ARGS
#define FHE_ARGS 3
#endif

namespace fhe
{
    class Node;
    typedef boost::intrusive_ptr< Node > NodePtr;
}

namespace boost
{
    void intrusive_ptr_add_ref( fhe::Node* node );
    void intrusive_ptr_release( fhe::Node* node );
}

namespace fhe
{
    
    class Node
    {
        public:
            typedef std::set< NodePtr >::const_iterator ChildIterator;
            
            friend void boost::intrusive_ptr_add_ref( Node* node );
            friend void boost::intrusive_ptr_release( Node* node );
            
        private:
            size_t m_refs;
            
            Node* m_parent;
            std::set< NodePtr > m_children;
            
            Node( const Node& n );
            void operator=( const Node& n );
            
        public:
            Node();
            virtual ~Node();
            
            NodePtr parent() const;
            NodePtr root() const;
            ChildIterator childrenBegin() const;
            ChildIterator childrenEnd() const;
            bool hasChild( const NodePtr& child );
            
            void attachToParent( const NodePtr& parent );
            void detachFromParent();
            void attachChild( const NodePtr& child );
            void detachChild( const NodePtr& child );
            
            template <class TObj, class TVar>
            TVar getVar( TVar (TObj::*var), TVar def = TVar() )
            {
                if ( TObj* t = dynamic_cast<TObj*>( this ) )
                {
                    return t->*var;
                }
                else
                {
                    return def;
                }
            }
            
            template <class TObj, class TVar>
            TVar getAncestorVar( TVar (TObj::*var), TVar def = TVar() )
            {
                if ( TObj* tparent = dynamic_cast<TObj*>( m_parent ) )
                {
                    return tparent->*var;
                }
                else if ( m_parent )
                {
                    return m_parent->getAncestorVar( var, def );
                }
                else
                {
                    return getVar( var, def );
                }
            }
            
            template <class TObj, class TVar>
            void setVar( TVar (TObj::*var), TVar val )
            {
                if ( TObj* t = dynamic_cast<TObj*>( this ) )
                {
                    t->*var = val;
                }
            }
            
            #define CALL_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
            
            #define CALL_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg ) > \
                TRet call( TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF( n ) \
                    BOOST_PP_ENUM( n, CALL_arg, ~ ), TRet def = TRet() ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) \
                        return (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    else \
                        return def; \
                }
            
            BOOST_PP_REPEAT( FHE_ARGS, CALL_iter, ~ )
            
            #undef CALL_iter
            
            #define CALLNORET_iter( z, n, unused ) \
                template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg ) > \
                void call( void (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF( n ) \
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) \
                        (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                }
            
            BOOST_PP_REPEAT( FHE_ARGS, CALLNORET_iter, ~ )
            
            #undef CALLNORET_iter
            
            #define PUBLISH_iter( z, n, unused ) \
                template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg ) > \
                void publish( void (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF( n ) \
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    call( func BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    for ( ChildIterator i = m_children.begin(); i != m_children.end(); ++i ) \
                        (*i)->publish( func BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, PUBLISH_iter, ~ )
            
            #undef PUBLISH_iter
            
            #undef CALL_arg
    };
    
}

#endif
