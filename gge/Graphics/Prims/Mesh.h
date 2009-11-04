#ifndef MESH_H
#define MESH_H

#include "Renderable.h"

namespace gge
{
    namespace Graphics
    {
    
        class Mesh : public Renderable
        {
            public:
                Mesh();
                
                Var on_attach( const Var& arg );
                
                Var set_meshName( const Var& val );
        };

    }
}

#endif
