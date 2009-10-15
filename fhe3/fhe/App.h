#ifndef APP_H
#define APP_H

#include "Aspect.h"

namespace fhe
{
    
    class App : public Aspect
    {
        public:
            App();
            
            float getTime();
            
            void run( float maxTime = -1 );
    };
    
}

#endif
