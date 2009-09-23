#ifndef MESH_H
#define MESH_H

#include "SceneNode.h"

namespace fhe
{
    
    class Mesh : public SceneNode
    {
        public:
            Mesh( const std::string& name, const std::string& type );
    
            void on_set_name( std::string name );
    };
    
    NODE_DECL(Mesh);
    
}

#endif
