#ifndef FHE_VAR_H
#define FHE_VAR_H

#include <fhe/Val.h>
#include <boost/shared_ptr.hpp>

namespace fhe
{
    
    class IVar
    {
        public:
            virtual Val get() const = 0;
            virtual void set( const Val& v ) = 0;
            virtual bool trySet( const Val& v ) = 0;
            virtual std::string name() const = 0;
    };
    
    typedef boost::shared_ptr<IVar> IVarPtr;
    
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
                FHE_ASSERT_MSG( v.is<TVar>(), "var type mismatch" );
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
