#ifndef PYASPECT_H
#define PYASPECT_H

#include "Aspect.h"

namespace gge
{
    
    class PyAspect : public Aspect
    {
        public:
            PyAspect();
            
            void load_script( TiXmlHandle h );
            
            void run( const std::string& filename );
    };
    
}

#endif
