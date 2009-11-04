#ifndef PYTHON_SCRIPT_H
#define PYTHON_SCRIPT_H

#include <gge/Aspect.h>
#include <boost/python.hpp>

namespace gge
{
    namespace Python
    {
        
        class Script : public Aspect
        {
            public:
                Script();
                
                Var runScript( const Var& arg );
                
                Var load_script( const Var& arg );
        };
        
    }
}

#endif
