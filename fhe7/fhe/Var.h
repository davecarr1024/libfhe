#ifndef FHE_VAR_H
#define FHE_VAR_H

#include <fhe/Val.h>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>

namespace fhe
{
    
    void deserialize( const YAML::Node& node, int& i );
    
    class IVar
    {
        public:
            virtual Val get() const = 0;
            virtual void set( const Val& v ) = 0;
            virtual bool trySet( const Val& v ) = 0;
            virtual std::string name() const = 0;
            virtual void deser( const YAML::Node& node ) = 0;
            virtual bool canSer() const = 0;
            virtual void ser( YAML::Emitter& out ) const = 0;
    };
    
    typedef boost::shared_ptr<IVar> IVarPtr;
    
    template <bool>
    struct deserialize_impl;
    
    template<>
    struct deserialize_impl<true>
    {
        template <class T>
        static void deser( const YAML::Node& node, T& t )
        {
            deserialize( node, t );
        }
    };
    
    template<>
    struct deserialize_impl<false>
    {
        template <class T>
        static void deser( const YAML::Node& node, T& t )
        {
            FHE_ERROR( "trying to deserialize unknown type %s", typeid(T).name() );
        }
    };
    
    namespace deserialize_fallback
    {
        struct flag { char c[2]; };
        flag deserialize( ... );
        
        int operator,( flag, flag );
        
        template <typename T>
        char operator,( flag, T const& );
        
        char operator,( int, flag );
        int operator,( char, flag );
    }
    
    template <class TObj, class TVar>
    class Var : public IVar
    {
        public:
            typedef TVar (TObj::*Ptr );
            
        private:
            std::string m_name;
            TObj* m_obj;
            Ptr m_ptr;
            
        public:
            Var( const std::string& name, TObj* obj, Ptr ptr ) :
                m_name( name ),
                m_obj( obj ),
                m_ptr( ptr )
            {
            }
            
            Val get() const
            {
                return Val( m_obj->*m_ptr );
            }
            
            void set( const Val& v )
            {
                FHE_ASSERT_MSG( v.is<TVar>(), "var type mismatch: expected %s got %s", 
                                typeid(TVar).name(), v.type().c_str() );
                m_obj->*m_ptr = v.get<TVar>();
            }
            
            bool trySet( const Val& v )
            {
                if ( v.is<TVar>() )
                {
                    m_obj->*m_ptr = v.get<TVar>();
                    return true;
                }
                else
                {
                    return false;
                }
            }
            
            std::string name() const
            {
                return m_name;
            }
            
            void deser( const YAML::Node& node )
            {
                using namespace deserialize_fallback;
                TVar& t = m_obj->*m_ptr;
                deserialize_impl
                    <sizeof( deserialize_fallback::flag(), deserialize( node, t ), deserialize_fallback::flag() ) != 1>
                    ::deser( node, t );
            }
            
            bool canSer() const
            {
                return false;
            }
            
            void ser( YAML::Emitter& out ) const
            {
            }
    };
    
    class Node;
    
    class IVarDesc
    {
        public:
            virtual IVarPtr build( Node* node ) = 0;
    };
    
    typedef boost::shared_ptr< IVarDesc > IVarDescPtr;
    
    template <class TObj, class TVar>
    class VarDesc : public IVarDesc
    {
        public:
            typedef TVar (TObj::*Ptr );
        
        private:
            std::string m_name;
            Ptr m_ptr;
            
        public:
            VarDesc( const std::string& name, Ptr ptr ) :
                m_name( name ),
                m_ptr( ptr )
            {
            }
            
            IVarPtr build( Node* node )
            {
                TObj* obj = dynamic_cast<TObj*>( node );
                FHE_ASSERT( obj );
                return IVarPtr( new Var<TObj,TVar>( m_name, obj, m_ptr ) );
            }
    };
    
}

#endif
