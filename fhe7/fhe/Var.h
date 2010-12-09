#ifndef FHE_VAR_H
#define FHE_VAR_H

#include <fhe/Val.h>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>

namespace fhe
{
    
    class IVar
    {
        public:
            virtual Val get() const = 0;
            virtual void set( const Val& v ) = 0;
            virtual bool trySet( const Val& v ) = 0;
            virtual std::string name() const = 0;
            virtual void deserialize( const YAML::Node& node ) = 0;
            virtual bool canSerialize() const = 0;
            virtual void serialize( YAML::Emitter& out ) const = 0;
    };
    
    typedef boost::shared_ptr<IVar> IVarPtr;
    
    template <class T>
    class TVar : public IVar
    {
        public:
            virtual T& var()=0;
            virtual const T& var() const = 0;
            
            void deserialize( const YAML::Node& node )
            {
                FHE_ERROR( "trying to deserialize %s", typeid(T).name() );
            }
            
            bool canSerialize() const
            {
                return false;
            }
            
            void serialize( YAML::Emitter& out ) const
            {
                FHE_ERROR( "trying to serialize %s", typeid(T).name() );
            }
    };
    
    #define TVAR_impl( type ) \
        template <> \
        class TVar<type> : public IVar \
        { \
            public: \
                virtual type& var()=0; \
                virtual const type& var() const = 0; \
                void deserialize( const YAML::Node& node ) \
                { \
                    node >> var(); \
                } \
                bool canSerialize() const \
                { \
                    return true; \
                } \
                void serialize( YAML::Emitter& out ) const \
                { \
                    out << var(); \
                } \
        };
        
    TVAR_impl( bool );
    TVAR_impl( int );
    TVAR_impl( double );
    TVAR_impl( std::string );
    
    template <class TObj, class T>
    class Var : public TVar<T>
    {
        public:
            typedef T (TObj::*Ptr );
            
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
            
            T& var()
            {
                return m_obj->*m_ptr;
            }
            
            const T& var() const
            {
                return m_obj->*m_ptr;
            }
            
            Val get() const
            {
                return Val( m_obj->*m_ptr );
            }
            
            void set( const Val& v )
            {
                FHE_ASSERT_MSG( v.is<T>(), "var type mismatch: expected %s got %s", 
                                typeid(T).name(), v.type().c_str() );
                m_obj->*m_ptr = v.get<T>();
            }
            
            bool trySet( const Val& v )
            {
                if ( v.is<T>() )
                {
                    m_obj->*m_ptr = v.get<T>();
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
