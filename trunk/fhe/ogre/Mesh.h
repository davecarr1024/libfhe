#ifndef MESH_H
#define MESH_H

#include "SceneNode.h"

namespace fhe
{
    
    class Mesh : public SceneNode
    {
        public:
            Mesh( const std::string& name, const std::string& type );
    
            void set_name( Var val );
    };
    
    NODE_DECL(Mesh);
    
}

#endif
