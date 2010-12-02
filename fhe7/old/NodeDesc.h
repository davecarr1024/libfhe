#ifndef FHE_NODEDESC_H
#define FHE_NODEDESC_H

#include <fhe/INodeDesc.h>
#include <vector>

namespace fhe
{
    
    template <class T>
    class NodeDesc : public INodeDesc
    {
        private:
            std::vector< IFuncDescPtr > m_funcs;
            std::string m_name;
            std::string m_parent;
            
        public:
            NodeDesc( const std::string& name ) :
                m_name( name )
            {
            }
            
            void addFunc( const IFuncDescPtr& func )
            {
                m_funcs.push_back( func );
            }
            
            NodePtr build()
            {
                return NodePtr( new T );
            }
            
            void init( Node* node )
            {
                if ( T* t = dynamic_cast<T*>( node ) )
                {
                    for ( std::vector< IFuncDescPtr >::const_iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
                    {
                        t->addFunc( (*i)->build( node ) );
                    }
                }
            }
            
            std::string name() const
            {
                return m_name;
            }
    };
    
}

#endif
