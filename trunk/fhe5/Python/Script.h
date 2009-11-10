#ifndef PYTHON_SCRIPT_H
#define PYTHON_SCRIPT_H

#include <fhe/Aspect.h>

namespace fhe
{
    namespace Python
    {
        
        class Script : public Aspect
        {
            public:
                FHE_FUNC_DECL(load_script);
                FHE_FUNC_DECL(runScript);
        };
        
    }
}

#endif
