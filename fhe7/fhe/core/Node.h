#ifndef FHE_NODE_H
#define FHE_NODE_H

#include <fhe/core/Var.h>
#include <fhe/core/Func.h>
#include <boost/intrusive_ptr.hpp>
#include <map>
#include <set>

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
    
    class INodeIntDesc;
    class PyNode;
    
    class Node
    {
        public:
            friend void boost::intrusive_ptr_add_ref( Node* node );
            friend void boost::intrusive_ptr_release( Node* node );

            typedef std::set< NodePtr >::const_iterator ChildrenIterator;
            typedef std::map< std::string, IFuncPtr >::const_iterator FuncIterator;
            typedef std::map< std::string, IVarPtr >::const_iterator VarIterator;
            
        private:
            friend class INodeIntDesc;
            friend class PyNode;
            
            size_t m_refs;
            
            std::string m_type;
            Node* m_parent;
            std::set< NodePtr > m_children;
            
            std::map< std::string, IFuncPtr > m_funcs;
            std::map< std::string, IVarPtr > m_vars;
            
            Node( const Node& n );
            void operator=( const Node& n );
            
            void addFunc( const IFuncPtr& func );
            void addVar( const IVarPtr& var );
            
        public:
            Node();
            virtual ~Node();
            
            static NodePtr load( const std::string& filename );
            void save( const std::string& filename ) const;
            
            std::string type() const;
            
            NodePtr parent() const;
            NodePtr root() const;
            bool hasChild( const NodePtr& child ) const;
            ChildrenIterator childrenBegin() const;
            ChildrenIterator childrenEnd() const;
            
            void attachToParent( const NodePtr& parent );
            void detachFromParent();
            void attachChild( const NodePtr& child );
            void detachChild( const NodePtr& child );
            
            bool hasVar( const std::string& name ) const;
            void setVar( const std::string& name, const Val& v );
            bool trySetVar( const std::string& name, const Val& v );
            Val getVar( const std::string& name ) const;
            bool tryGetVar( const std::string& name, Val& v ) const;
            bool getAncestorVar( const std::string& name, Val& v ) const;
            VarIterator varsBegin() const;
            VarIterator varsEnd() const;
            IVarPtr getIVar( const std::string& name ) const;
            
            template <class TObj, class TVar>
            TVar getVar( TVar (TObj::*ptr) ) const
            {
                const TObj* t = dynamic_cast<const TObj*>( this );
                FHE_ASSERT_MSG( t, "unable to cast node to type %s", typeid(TObj).name() );
                return t->*ptr;
            }
            
            template <class TObj, class TVar>
            void setVar( TVar (TObj::*ptr), TVar val )
            {
                FHE_ASSERT_MSG( trySetVar( ptr, val ),
                                "unable to set var for type %s", typeid(TObj).name() );
            }
            
            template <class TObj, class TVar>
            bool trySetVar( TVar (TObj::*ptr), TVar val )
            {
                if ( TObj* t = dynamic_cast<TObj*>( this ) )
                {
                    t->*ptr = val;
                    return true;
                }
                else
                {
                    return false;
                }
            }
            
            template <class TObj, class TVar>
            bool hasVar( TVar (TObj::*ptr) ) const
            {
                return dynamic_cast<const TObj*>( this );
            }
            
            template <class TObj, class TVar>
            bool tryGetVar( TVar (TObj::*ptr), TVar& v ) const
            {
                if ( const TObj* t = dynamic_cast<const TObj*>( this ) )
                {
                    v = t->*ptr;
                    return true;
                }
                else
                {
                    return false;
                }
            }
            
            template <class TObj, class TVar>
            bool getAncestorVar( TVar (TObj::*ptr), TVar& v ) const
            {
                if ( m_parent )
                {
                    if ( m_parent->tryGetVar( ptr, v ) )
                    {
                        return true;
                    }
                    else
                    {
                        return m_parent->getAncestorVar( ptr, v );
                    }
                }
                else
                {
                    return false;
                }
            }
            
            bool hasFunc( const std::string& name ) const;
            IFuncPtr getFunc( const std::string& name ) const;
            FuncIterator funcsBegin() const;
            FuncIterator funcsEnd() const;
            
            Val call( const std::string& name, const std::vector< Val >& args );
            bool tryCall( const std::string& name, const std::vector< Val >& args, Val& ret );
            void publish( const std::string& name, const std::vector< Val >& args );
            
            //TRet call( &Class::Func, args );
            
            #define CALL_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
            
            #define CALL_iter( z, n, unused ) \
                template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                void call( void (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    TObj* t = dynamic_cast<TObj*>( this ); \
                    FHE_ASSERT_MSG( t, "unable to cast node to type %s", typeid(TObj).name() ); \
                    (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, CALL_iter, ~ )
            
            #undef CALL_iter
            
            #define RETCALL_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                TRet call( TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    TObj* t = dynamic_cast<TObj*>( this ); \
                    FHE_ASSERT_MSG( t, "unable to cast node to type %s", typeid(TObj).name() ); \
                    return (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, RETCALL_iter, ~ )
            
            #undef RETCALL_iter
            
            #define TRYCALL_iter( z, n, unused ) \
                template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                bool tryCall( void (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) { \
                        (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                        return true; \
                    } else {\
                        return false; \
                    }\
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, TRYCALL_iter, ~ )
            
            //bool tryCall( &Class::Func, args, TRet& ret );
            
            #undef TRYCALL_iter
            
            #define RETTRYCALL_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                bool tryCall( TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ), TRet& ret ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) { \
                        ret = (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                        return true; \
                    } else {\
                        return false; \
                    }\
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, RETTRYCALL_iter, ~ )
            
            #undef RETTRYCALL_iter
            
            //void publish( &Class::Func, args );

            #define PUBLISH_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                void publish( TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) { \
                        (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    } \
                    for ( ChildrenIterator i = m_children.begin(); i != m_children.end(); ++i ) { \
                        (*i)->publish( func BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                    } \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, PUBLISH_iter, ~ )
            
            #undef PUBLISH_iter
            
            //bool ancestorCall( Class::Func, args, ret );
            
            #define ANCESTORCALL_iter( z, n, unused ) \
                template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                bool ancestorCall( void (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
                    if ( m_parent ) { \
                        if ( TObj* t = dynamic_cast<TObj*>( m_parent ) )  { \
                            (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                            return true; \
                        } else { \
                            return m_parent->ancestorCall( func BOOST_PP_COMMA_IF(n) BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                        } \
                    } else { \
                        return false; \
                    } \
                }

            BOOST_PP_REPEAT( FHE_ARGS, ANCESTORCALL_iter, ~ )
            
            #undef ANCESTORCALL_iter

            #define RETANCESTORCALL_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                bool ancestorCall( TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) BOOST_PP_COMMA_IF(n)\
                    BOOST_PP_ENUM( n, CALL_arg, ~ ), TRet& ret ) { \
                    if ( m_parent ) { \
                        if ( TObj* t = dynamic_cast<TObj*>( m_parent ) )  { \
                            ret = (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                            return true; \
                        } else { \
                            return m_parent->ancestorCall( func, BOOST_PP_ENUM_PARAMS( n, arg ) BOOST_PP_COMMA_IF(n) ret ); \
                        } \
                    } else { \
                        return false; \
                    } \
                }

            BOOST_PP_REPEAT( FHE_ARGS, RETANCESTORCALL_iter, ~ )
            
            #undef RETANCESTORCALL_iter

            #undef CALL_arg
    };
    
    class FuncRegisterer;
    class VarRegisterer;
    class DepRegisterer;
    
    class INodeDesc;
    typedef boost::shared_ptr< INodeDesc > INodeDescPtr;
    
    class INodeIntDesc;
    typedef boost::shared_ptr< INodeIntDesc > INodeIntDescPtr;
    
    class INodeIntDesc
    {
        private:
            friend class FuncRegisterer;
            friend class VarRegisterer;
            friend class DepRegisterer;
            
            std::string m_name;
            std::vector< IFuncDescPtr > m_funcs;
            std::vector< IVarDescPtr > m_vars;
            std::vector< INodeIntDescPtr > m_deps;
            
            void addFunc( const IFuncDescPtr& func );
            void addVar( const IVarDescPtr& var );
            void addDep( const INodeIntDescPtr& dep );
            
        protected:
            INodeIntDesc( const std::string& name );
            
        public:
            bool isDep( const INodeIntDescPtr& dep ) const;
            
            virtual bool canInit( Node* node ) const = 0;
            
            void init( Node* node ) const;
            
            std::string name() const;
    };
    
    template <class T>
    class NodeIntDesc : public INodeIntDesc
    {
        public:
            NodeIntDesc( const std::string& name ) :
                INodeIntDesc( name )
            {
            }
            
            bool canInit( Node* node ) const
            {
                return dynamic_cast<T*>( node );
            }
    };
    
    class INodeDesc : public INodeIntDesc
    {
        protected:
            INodeDesc( const std::string& name );
            
        public:
            virtual Node* build() const = 0;
    };
    
    template <class T>
    class NodeDesc : public INodeDesc
    {
        public:
            NodeDesc( const std::string& name ) :
                INodeDesc( name )
            {
            }
            
            Node* build() const
            {
                return new T;
            }
            
            bool canInit( Node* node ) const
            {
                return dynamic_cast<T*>( node );
            }
    };
    
    void operator>>( const YAML::Node& doc, NodePtr& node );
    YAML::Emitter& operator<<( YAML::Emitter& out, const NodePtr& node );
    
}

#include <fhe/core/NodeFactory.h>

#endif

