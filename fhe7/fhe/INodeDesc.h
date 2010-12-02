#ifndef FHE_INODEDESC_H
#define FHE_INODEDESC_H

#include <fhe/Node.h>
#include <fhe/IFuncDesc.h>
#include <typeinfo>

namespace fhe
{
    
    class INodeDesc
    {
        public:
            virtual NodePtr build() = 0;
            virtual void init( Node* ) = 0;
            virtual void addFunc( const IFuncDescPtr& func ) = 0;
            virtual std::string name() const = 0;
    };
    
    typedef boost::shared_ptr< INodeDesc > INodeDescPtr;
    
}

#endif
