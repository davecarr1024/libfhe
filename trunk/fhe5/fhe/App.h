#ifndef APP_H
#define APP_H

#include <fhe/Aspect.h>

namespace fhe
{
    class App : public Aspect
    {
        public:
            FHE_FUNC_DECL(run);
            FHE_FUNC_DECL(shutdown);
            FHE_FUNC_DECL(get_time);
    };
}

#endif
