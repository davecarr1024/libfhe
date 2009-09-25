#ifndef APP_H
#define APP_H

#include "Node.h"

namespace fhe
{
    
    class App : public Node
    {
        public:
            App();
            
            float getTime();
            
            void run( float maxTime = -1 );
    };
    
    FHE_NODE_DECL(App);
    
}

#endif
