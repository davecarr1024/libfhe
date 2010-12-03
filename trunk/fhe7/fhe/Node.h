#ifndef FHE_NODE_H
#define FHE_NODE_H

#include <fhe/Var.h>
#include <fhe/Func.h>
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
    
    class Node
    {
        public:
            friend void boost::intrusive_ptr_add_ref( Node* node );
            friend void boost::intrusive_ptr_release( Node* node );

            typedef std::set< NodePtr >::const_iterator ChildrenIterator;
            
        private:
            friend class INodeIntDesc;
            
            size_t m_refs;
            
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
                if ( tryGetVar( ptr, v ) )
                {
                    return true;
                }
                else if ( m_parent )
                {
                    return m_parent->getAncestorVar( ptr, v );
                }
                else
                {
                    return false;
                }
            }
            
            Val call( const std::string& name, const std::vector< Val >& args );
            
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
                    BOOST_PP_ENUM( n, RETCALL_arg, ~ ) ) { \
                    TObj* t = dynamic_cast<TObj*>( this ); \
                    FHE_ASSERT_MSG( t, "unable to cast node to type %s", typeid(TObj).name() ); \
                    return (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg ) ); \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, RETCALL_iter, ~ )
            
            #undef RETCALL_iter
            #undef RETCALL_arg
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
            
            virtual bool canInit( Node* node ) const=0;
            
            virtual void init( Node* node ) const;
            
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
    
}

#endif
