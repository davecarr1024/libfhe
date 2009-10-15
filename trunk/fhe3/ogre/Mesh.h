#ifndef MESH_H
#define MESH_H

#include "SceneNode.h"

namespace fhe
{
    
    class Mesh : public SceneNode
    {
        public:
            Mesh();
    
            void set_name( Var val );
    };
    
}

#endif
