#ifndef FHE_IFUNCDESC_H
#define FHE_IFUNCDESC_H

#include <fhe/IFunc.h>

namespace fhe
{
    
    class IFuncDesc
    {
        public:
            virtual IFuncPtr build( Node* node ) = 0;
    };
    
    typedef boost::shared_ptr< IFuncDesc > IFuncDescPtr;
    
}

#endif
