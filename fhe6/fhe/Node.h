#ifndef FHE_NODE_H
#define FHE_NODE_H

#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>
#include <boost/intrusive_ptr.hpp>
#include <string>
#include <map>

namespace fhe
{
    class Node;
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
            typedef boost::intrusive_ptr< Node > Ptr;
            typedef std::map< std::string, Ptr >::const_iterator ChildIterator;
            
            friend void boost::intrusive_ptr_add_ref( Node* node );
            friend void boost::intrusive_ptr_release( Node* node );
            
        private:
            size_t m_refs;
            
            std::string m_name;
            
            Node* m_parent;
            
            std::map< std::string, Ptr > m_children;
            
            Ptr getLocalNode( const std::string& path ) const;
            
            Node( const Node& n );
            void operator=( const Node& n );
            
        public:
            Node( const std::string& name );
            virtual ~Node();
            
            virtual void onAttach() {}
            virtual void onDetach() {}
            
            std::string name() const;
            std::string path() const;
            Ptr parent() const;
            Ptr root() const;
            Ptr getNode( const std::string& path ) const;
            bool hasChild( const std::string& name ) const;
            Ptr getChild( const std::string& name ) const;
            ChildIterator childBegin() const;
            ChildIterator childEnd() const;
            
            void attachToParent( const Ptr& parent );
            void detachFromParent();
            void attachChild( const Ptr& child );
            void detachChild( const Ptr& child );
            
            template <class T>
            boost::intrusive_ptr<T> as() const
            {
                return boost::dynamic_pointer_cast<T,Node>( const_cast<Node*>( this ) );
            }
            
            template <class T>
            boost::intrusive_ptr<T> parent() const
            {
                if ( boost::intrusive_ptr<T> t = m_parent->as<T>() )
                {
                    return t;
                }
                else
                {
                    return m_parent->parent<T>();
                }
            }
            
            #ifndef PUBLISH_ARGS
            #define PUBLISH_ARGS 5
            #endif
            
            #define PUBLISH_arg( z, n, unused ) BOOST_PP_CAT( TArg, n ) BOOST_PP_CAT( arg, n )
            
            #define PUBLISH_iter( z, n, unused ) \
                template <class TObj BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg) > \
                void publish( void (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg) ) BOOST_PP_COMMA_IF( n ) \
                    BOOST_PP_ENUM( n, PUBLISH_arg, ~ ) ) { \
                    if ( TObj* t = dynamic_cast<TObj*>( this ) ) { \
                        (t->*func)( BOOST_PP_ENUM_PARAMS( n, arg) ); \
                    } \
                    for ( ChildIterator i = m_children.begin(); i != m_children.end(); ++i ) \
                        i->second->publish( func BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, arg) ); \
                }

            BOOST_PP_REPEAT( PUBLISH_ARGS, PUBLISH_iter, ~ )
                  
            #undef PUBLISH_arg
            #undef PUBLISH_iter
    };
    
}

#endif
