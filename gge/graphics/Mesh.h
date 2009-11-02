#ifndef MESH_H
#define MESH_H

#include "Renderable.h"

namespace gge
{
    
    class Mesh : public Renderable
    {
        public:
            Mesh();
            
            void on_attach();
            
            void set_name( Var val );
    };
    
}

#endif
