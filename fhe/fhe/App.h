#ifndef APP_H
#define APP_H

#include "Node.h"

namespace fhe
{
    
    class App : public Node
    {
        public:
            App( const std::string& type, const std::string& name );
            
            float getTime();
            
            void run( float maxTime = -1 );
    };
    
    NODE_DECL(App);
    
}

#endif
