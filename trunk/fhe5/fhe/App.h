#ifndef APP_H
#define APP_H

#include <fhe/Aspect.h>

namespace fhe
{
    class App : public Aspect
    {
        public:
            FHE_FUNC_DECL(App,run);
            FHE_FUNC_DECL(App,shutdown);
            FHE_FUNC_DECL(App,get_time);
    };
}

#endif
